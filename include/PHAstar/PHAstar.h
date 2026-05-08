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

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>

// PHA* Implementation

extern bool DEBUG_VIS;

struct ReferenceExperienceGraphOptions
{
    bool enabled = false;
    double epsilon = 10.0;
    double waypoint_spacing = 0.10;
    double snap_radius = 0.25;
    double snap_yaw = M_PI / 3.0;
    int successor_lookahead = 3;
    int max_nodes = 120;
};

struct ReferenceExperienceNode
{
    Pose pose;
    double suffix_cost = 0.0;
};

struct ReferenceExperienceGraph
{
    bool enabled = false;
    ReferenceExperienceGraphOptions options;
    std::vector<ReferenceExperienceNode> nodes;
};

class PHAStarExpansionWorkerPool
{
public:
    explicit PHAStarExpansionWorkerPool(int total_threads)
    {
        int worker_count = std::max(0, total_threads - 1);
        workers_.reserve(static_cast<std::size_t>(worker_count));
        for (int i = 0; i < worker_count; ++i)
        {
            workers_.emplace_back([this]()
                                  { worker_loop(); });
        }
    }

    ~PHAStarExpansionWorkerPool()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        task_cv_.notify_all();
        for (auto &worker : workers_)
        {
            if (worker.joinable())
                worker.join();
        }
    }

    std::size_t total_threads() const
    {
        return workers_.size() + 1;
    }

    bool parallel_enabled(std::size_t work_count) const
    {
        return !workers_.empty() && work_count > 1;
    }

    template <typename Func>
    void parallel_for(std::size_t work_count, Func &&func)
    {
        if (!parallel_enabled(work_count))
        {
            for (std::size_t i = 0; i < work_count; ++i)
                func(i);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            job_count_ = work_count;
            next_index_ = 0;
            active_workers_ = workers_.size();
            first_exception_ = nullptr;
            job_fn_ = [&func](std::size_t index)
            {
                func(index);
            };
            has_job_ = true;
            ++job_generation_;
        }

        task_cv_.notify_all();
        consume_work();

        std::exception_ptr captured_exception;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            done_cv_.wait(lock, [this]()
                          { return active_workers_ == 0; });
            captured_exception = first_exception_;
            has_job_ = false;
            job_fn_ = nullptr;
        }

        if (captured_exception)
            std::rethrow_exception(captured_exception);
    }

private:
    std::vector<std::thread> workers_;
    mutable std::mutex mutex_;
    std::condition_variable task_cv_;
    std::condition_variable done_cv_;
    bool stop_ = false;
    bool has_job_ = false;
    std::size_t job_generation_ = 0;
    std::size_t job_count_ = 0;
    std::size_t next_index_ = 0;
    std::size_t active_workers_ = 0;
    std::function<void(std::size_t)> job_fn_;
    std::exception_ptr first_exception_;

    bool take_index(std::size_t &index)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_job_ || next_index_ >= job_count_)
            return false;
        index = next_index_++;
        return true;
    }

    void store_exception(std::exception_ptr ex)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!first_exception_)
            first_exception_ = ex;
        next_index_ = job_count_;
    }

    void consume_work()
    {
        while (true)
        {
            std::size_t index = 0;
            if (!take_index(index))
                break;

            try
            {
                job_fn_(index);
            }
            catch (...)
            {
                store_exception(std::current_exception());
                break;
            }
        }
    }

    void worker_loop()
    {
        std::size_t observed_generation = 0;
        while (true)
        {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                task_cv_.wait(lock, [this, &observed_generation]()
                              { return stop_ || (has_job_ && job_generation_ != observed_generation); });
                if (stop_)
                    return;
                observed_generation = job_generation_;
            }

            consume_work();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (active_workers_ > 0)
                    --active_workers_;
                if (active_workers_ == 0)
                    done_cv_.notify_one();
            }
        }
    }
};

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
    EntityMeta *terminal_contact_entity = nullptr;
    bool is_transfer;
    bool ignore_other_robots = false; // New flag for ghost planning
    bool suppress_debug_popup = false;
    std::string debug_plan_kind;
    Params params;
    int x_width, y_width, theta_width, time_width;
    std::vector<double> entity_diags;
    ReferenceExperienceGraph reference_egraph;

    // Backup path (valid geometry but blocked by relocatable robot)
    PlanningResult backup_result;

    // Search limits
    int max_search_iterations = 20000;
    int planner_expansion_threads = 1;

    // For diagnostics
    double max_planning_time = 10.0; // [s] timeout threshold (tune via params if needed)
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock;

    size_t calc_grid_index(const Node *node)
    {
        int ix = static_cast<int>(std::floor((node->x - params.min_x) / params.xy_resolution));
        int iy = static_cast<int>(std::floor((node->y - params.min_y) / params.xy_resolution));
        double normalized_yaw = mod2pi(node->yaw);
        int itheta = static_cast<int>(std::floor(normalized_yaw / params.yaw_resolution));
        int itime = static_cast<int>(std::floor(node->t / params.time_resolution));
        ix = std::max(0, std::min(ix, x_width - 1));
        iy = std::max(0, std::min(iy, y_width - 1));
        itheta = std::max(0, std::min(itheta, theta_width - 1));
        itime = std::max(0, std::min(itime, time_width - 1));
        return (((static_cast<size_t>(iy) * x_width + ix) * theta_width + itheta) * time_width) + itime;
    }

    Node *new_node(double x, double y, double yaw, double t, double cost, double steer, Node *parent, int direction)
    {
        all_nodes.emplace_back(std::make_unique<Node>(x, y, mod2pi(yaw), t, cost, steer, parent, direction));
        return all_nodes.back().get();
    }

    int collision_samples_for_duration(double duration) const
    {
        const double step = std::max(1e-3, params.collision_check_time_step);
        return std::max(1, static_cast<int>(std::ceil(duration / step)));
    }

    double pose_distance(const Pose &a, const Pose &b) const
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double yaw_distance(double a, double b) const
    {
        return std::fabs(pi_2_pi(a - b));
    }

    double reference_transition_cost(const Pose &from, const Pose &to) const
    {
        return pose_distance(from, to) +
               0.1 * min_turn_radius * yaw_distance(from.yaw, to.yaw);
    }

    void rebuild_reference_suffix_costs()
    {
        if (reference_egraph.nodes.empty())
            return;

        reference_egraph.nodes.back().suffix_cost = 0.0;
        for (std::size_t idx = reference_egraph.nodes.size() - 1; idx > 0; --idx)
        {
            const Pose &from = reference_egraph.nodes[idx - 1].pose;
            const Pose &to = reference_egraph.nodes[idx].pose;
            reference_egraph.nodes[idx - 1].suffix_cost =
                reference_egraph.nodes[idx].suffix_cost +
                reference_transition_cost(from, to);
        }
    }

    void build_reference_egraph(const std::vector<Waypoint> &reference_waypoints,
                                const ReferenceExperienceGraphOptions &options)
    {
        reference_egraph = ReferenceExperienceGraph{};
        reference_egraph.options = options;
        if (is_transfer || !options.enabled || reference_waypoints.size() < 2)
            return;

        const int max_nodes = std::max(2, options.max_nodes);
        const double spacing = std::max(1e-6, options.waypoint_spacing);
        reference_egraph.nodes.reserve(
            std::min<std::size_t>(reference_waypoints.size(),
                                  static_cast<std::size_t>(max_nodes)));

        auto push_pose = [&](const Waypoint &wp)
        {
            ReferenceExperienceNode node;
            node.pose = Pose(wp.x, wp.y, mod2pi(wp.yaw));
            reference_egraph.nodes.push_back(node);
        };

        push_pose(reference_waypoints.front());
        Pose last_pose(reference_waypoints.front().x,
                       reference_waypoints.front().y,
                       mod2pi(reference_waypoints.front().yaw));
        double accumulated = 0.0;

        for (std::size_t i = 1; i < reference_waypoints.size(); ++i)
        {
            Pose pose(reference_waypoints[i].x, reference_waypoints[i].y,
                      mod2pi(reference_waypoints[i].yaw));
            accumulated += pose_distance(last_pose, pose);
            const bool is_last = (i + 1 == reference_waypoints.size());
            if ((accumulated >= spacing || is_last) &&
                reference_egraph.nodes.size() <
                    static_cast<std::size_t>(max_nodes))
            {
                push_pose(reference_waypoints[i]);
                accumulated = 0.0;
            }
            last_pose = pose;
        }

        const Waypoint &last_wp = reference_waypoints.back();
        const Pose endpoint(last_wp.x, last_wp.y, mod2pi(last_wp.yaw));
        const Pose stored_endpoint = reference_egraph.nodes.back().pose;
        if (pose_distance(endpoint, stored_endpoint) > 1e-6 ||
            yaw_distance(endpoint.yaw, stored_endpoint.yaw) > 1e-6)
        {
            if (reference_egraph.nodes.size() >= static_cast<std::size_t>(max_nodes))
                reference_egraph.nodes.back().pose = endpoint;
            else
                push_pose(last_wp);
        }

        rebuild_reference_suffix_costs();
        reference_egraph.enabled = reference_egraph.nodes.size() >= 2;
    }

    std::vector<std::unique_ptr<Node>> all_nodes; // To manage memory

    double speed;
    double max_steer;
    double max_curvature;
    double wheel_base;
    double min_turn_radius;
    std::vector<std::pair<int, double>> motion_primitives;

    bool compute_generated_node(const Node *current, std::pair<int, double> prim,
                                Node &out_node) const
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
            return false;
        double cost = current->cost + additional_cost + distance;
        out_node = Node(x, y, mod2pi(yaw), t, cost, steer,
                        const_cast<Node *>(current), direction);
        return true;
    }

    Node *generate_node(Node *current, std::pair<int, double> prim)
    {
        Node generated;
        if (!compute_generated_node(current, prim, generated))
            return nullptr;
        return new_node(generated.x, generated.y, generated.yaw, generated.t,
                        generated.cost, generated.steer, current,
                        generated.direction);
    }

    Waypoint waypoint_from_node(const Node *node) const
    {
        Waypoint w;
        w.time = node->t;
        w.x = node->x;
        w.y = node->y;
        w.yaw = node->yaw;
        w.linear_velocity = node->direction * speed;
        w.steering_angle = node->steer;
        return w;
    }

    void append_waypoint_if_distinct(std::vector<Waypoint> &waypoints,
                                     const Waypoint &w) const
    {
        if (!waypoints.empty())
        {
            const Waypoint &last = waypoints.back();
            if (std::abs(last.time - w.time) < 1e-9 &&
                std::hypot(last.x - w.x, last.y - w.y) < 1e-9 &&
                yaw_distance(last.yaw, w.yaw) < 1e-9)
            {
                return;
            }
        }
        waypoints.push_back(w);
    }

    bool is_motion_primitive_link(const Node *parent, const Node *child) const
    {
        if (!parent || !child)
            return false;
        const double dt = child->t - parent->t;
        if (dt <= 1e-9)
            return false;
        return std::abs(dt - params.time_step) <=
               std::max(1e-6, params.time_step * 1e-6);
    }

    void append_sampled_motion_primitive(std::vector<Waypoint> &waypoints,
                                         const Node *parent,
                                         const Node *child) const
    {
        if (!parent || !child)
            return;

        const double dt = std::max(0.0, child->t - parent->t);
        if (dt <= 1e-9)
            return;

        const int direction = child->direction;
        const double steer = child->steer;
        if (direction == 0)
        {
            append_waypoint_if_distinct(waypoints, waypoint_from_node(child));
            return;
        }

        const int samples = std::max(params.collision_steps,
                                     collision_samples_for_duration(dt));

        for (int i = 1; i <= samples; ++i)
        {
            const double frac = static_cast<double>(i) / samples;
            double x_i = parent->x;
            double y_i = parent->y;
            double yaw_i = parent->yaw;

            {
                const double steer_adjusted = (direction > 0) ? steer : -steer;
                const double d = direction * (frac * std::abs(speed * dt));
                if (std::abs(steer_adjusted) < 1e-5)
                {
                    x_i += d * std::cos(yaw_i);
                    y_i += d * std::sin(yaw_i);
                }
                else
                {
                    const double R = wheel_base / std::tan(steer_adjusted);
                    const double beta = d / R;
                    x_i += R * (std::sin(yaw_i + beta) - std::sin(yaw_i));
                    y_i += R * (std::cos(yaw_i) - std::cos(yaw_i + beta));
                    yaw_i = mod2pi(yaw_i + beta);
                }
            }

            Waypoint w;
            w.time = parent->t + frac * dt;
            w.x = x_i;
            w.y = y_i;
            w.yaw = mod2pi(yaw_i);
            w.linear_velocity = direction * speed;
            w.steering_angle = steer;
            append_waypoint_if_distinct(waypoints, w);
        }
    }

    std::vector<Waypoint> extract_parent_chain_path(Node *node) const
    {
        std::vector<Node *> chain;
        for (Node *current = node; current; current = current->parent)
            chain.push_back(current);
        std::reverse(chain.begin(), chain.end());

        std::vector<Waypoint> waypoints;
        if (chain.empty())
            return waypoints;

        waypoints.reserve(chain.size());
        append_waypoint_if_distinct(waypoints, waypoint_from_node(chain.front()));
        for (std::size_t i = 1; i < chain.size(); ++i)
        {
            Node *parent = chain[i - 1];
            Node *child = chain[i];
            if (is_motion_primitive_link(parent, child))
            {
                append_sampled_motion_primitive(waypoints, parent, child);
            }
            else
            {
                append_waypoint_if_distinct(waypoints, waypoint_from_node(child));
            }
        }
        return waypoints;
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
                if (it_pose != poses.end() &&
                    is_terminal_approach_contact_pose(
                        robot_pose, collision_result.colliding_entity,
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
        int samples = std::max(params.collision_steps,
                               collision_samples_for_duration(time_inc));

        for (int i = 1; i <= samples; ++i)
        {
            double frac = static_cast<double>(i) / samples;
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
                    yaw_i = mod2pi(yaw_i + beta);
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
        auto candidates = analytic_expand_all(node);
        if (candidates.empty())
            return {{}, 0.0};
        const auto &best_path = candidates.front();
        std::vector<std::tuple<double, double, double>> rs_path;
        for (size_t i = 0; i < best_path.x.size(); ++i)
        {
            rs_path.emplace_back(best_path.x[i], best_path.y[i], best_path.yaw[i]);
        }
        return {rs_path, best_path.L};
    }

    std::vector<ReedShepp::Path> analytic_expand_all(Node *node)
    {
        double dx = goal->x - node->x;
        double dy = goal->y - node->y;
        double dist = std::hypot(dx, dy);
        if (dist > params.analytic_threshold)
            return {};

        auto paths = ReedShepp::calc_paths(node->x, node->y, node->yaw,
                                           goal->x, goal->y, goal->yaw,
                                           max_curvature, params.rs_step_size,
                                           wheel_base);
        std::sort(paths.begin(), paths.end(),
                  [](const ReedShepp::Path &a, const ReedShepp::Path &b)
                  {
                      return std::abs(a.L) < std::abs(b.L);
                  });
        paths.erase(std::remove_if(paths.begin(), paths.end(),
                                   [&](const ReedShepp::Path &path)
                                   {
                                       return !analytic_path_allowed_for_mode(path);
                                   }),
                    paths.end());
        return paths;
    }

    std::vector<std::tuple<double, double, double>> rs_points(const ReedShepp::Path &path) const
    {
        std::vector<std::tuple<double, double, double>> points;
        points.reserve(path.x.size());
        for (size_t i = 0; i < path.x.size(); ++i)
        {
            points.emplace_back(path.x[i], path.y[i], path.yaw[i]);
        }
        return points;
    }

    bool analytic_path_allowed_for_mode(const ReedShepp::Path &path) const
    {
        if (!is_transfer)
            return true;

        // Transfer mode pushes an object, so analytic shortcuts must not use
        // reverse motion even though Reed-Shepp can generate it.
        for (int direction : path.directions)
        {
            if (direction < 0)
                return false;
        }
        for (double length : path.lengths)
        {
            if (length < -1e-9)
                return false;
        }
        return true;
    }

    double rs_arrival_time(const Node *node, const ReedShepp::Path &rs_path) const
    {
        double rs_t = node->t;
        double speed_val = speed;
        if (speed_val <= 1e-6)
            speed_val = 0.1;
        for (size_t i = 1; i < rs_path.x.size(); ++i)
        {
            const double dist = std::hypot(rs_path.x[i] - rs_path.x[i - 1],
                                           rs_path.y[i] - rs_path.y[i - 1]);
            rs_t += dist / speed_val;
        }
        return rs_t;
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
                int samples = std::max(params.collision_steps,
                                       collision_samples_for_duration(time_inc));
                for (int j = 1; j <= samples; ++j)
                {
                    double frac = static_cast<double>(j) / samples;
                    double x_i = x1 + frac * dx;
                    double y_i = y1 + frac * dy;

                    // Interpolate Angle correctly
                    double dyaw = yaw2 - yaw1;
                    while (dyaw > M_PI)
                        dyaw -= 2 * M_PI;
                    while (dyaw < -M_PI)
                        dyaw += 2 * M_PI;
                    double yaw_i = mod2pi(yaw1 + frac * dyaw);

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

    double calc_base_heuristic_to_pose(const Node *node, const Pose &target) const
    {
        auto [_, __, ___, ____, lengths, _____, ______] =
            ReedShepp::reeds_shepp_path_planning(
                node->x, node->y, node->yaw, target.x, target.y, target.yaw,
                max_curvature, params.rs_step_size * 10, wheel_base);
        if (lengths.empty())
            return std::numeric_limits<double>::infinity();

        double h = 0.0;
        for (double l : lengths)
            h += std::abs(l);
        return h;
    }

    double calc_anchor_heuristic(Node *node) const
    {
        const Pose goal_pose(goal->x, goal->y, goal->yaw);
        return calc_base_heuristic_to_pose(node, goal_pose);
    }

    double calc_reference_egraph_heuristic(Node *node) const
    {
        const Pose goal_pose(goal->x, goal->y, goal->yaw);
        const double base_h = calc_anchor_heuristic(node);
        if (!reference_egraph.enabled || reference_egraph.nodes.empty())
            return base_h;

        const double epsilon = std::max(1.0, reference_egraph.options.epsilon);
        double best_h = std::isfinite(base_h)
                            ? epsilon * base_h
                            : std::numeric_limits<double>::infinity();

        std::vector<std::pair<double, std::size_t>> ranked_refs;
        ranked_refs.reserve(reference_egraph.nodes.size());
        for (std::size_t idx = 0; idx < reference_egraph.nodes.size(); ++idx)
        {
            const Pose &pose = reference_egraph.nodes[idx].pose;
            const double dist = std::hypot(node->x - pose.x, node->y - pose.y);
            const double yaw_err = yaw_distance(node->yaw, pose.yaw);
            ranked_refs.emplace_back(dist + 0.1 * min_turn_radius * yaw_err,
                                     idx);
        }

        constexpr std::size_t kMaxReferenceHeuristicTargets = 8;
        const std::size_t eval_count =
            std::min(kMaxReferenceHeuristicTargets, ranked_refs.size());
        if (ranked_refs.size() > eval_count)
        {
            std::partial_sort(
                ranked_refs.begin(), ranked_refs.begin() + eval_count,
                ranked_refs.end(),
                [](const auto &a, const auto &b)
                { return a.first < b.first; });
        }
        else
        {
            std::sort(ranked_refs.begin(), ranked_refs.end(),
                      [](const auto &a, const auto &b)
                      { return a.first < b.first; });
        }

        for (std::size_t rank = 0; rank < eval_count; ++rank)
        {
            const auto &ref_node = reference_egraph.nodes[ranked_refs[rank].second];
            const double h_to_ref =
                calc_base_heuristic_to_pose(node, ref_node.pose);
            if (!std::isfinite(h_to_ref))
                continue;

            const double h = epsilon * h_to_ref + ref_node.suffix_cost;
            if (h < best_h)
                best_h = h;
        }

        return best_h;
    }

    double calc_heuristic(Node *node) const
    {
        return calc_anchor_heuristic(node);
    }

    int nearest_reference_index(const Node *node,
                                double radius_override = -1.0,
                                double yaw_override = -1.0) const
    {
        if (!reference_egraph.enabled)
            return -1;

        const double max_dist =
            std::max(0.0, radius_override >= 0.0
                              ? radius_override
                              : reference_egraph.options.snap_radius);
        const double max_yaw =
            std::max(0.0, yaw_override >= 0.0
                              ? yaw_override
                              : reference_egraph.options.snap_yaw);
        int best_idx = -1;
        double best_score = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < reference_egraph.nodes.size(); ++i)
        {
            const Pose &pose = reference_egraph.nodes[i].pose;
            const double dist = std::hypot(node->x - pose.x, node->y - pose.y);
            if (dist > max_dist)
                continue;
            const double yaw_err = yaw_distance(node->yaw, pose.yaw);
            if (yaw_err > max_yaw)
                continue;
            const double score = dist + 0.1 * min_turn_radius * yaw_err;
            if (score < best_score)
            {
                best_score = score;
                best_idx = static_cast<int>(i);
            }
        }
        return best_idx;
    }

    struct ReferenceSuccessorCandidate
    {
        Node node;
        bool is_snap = false;
        ReedShepp::Path local_path;
    };

    std::vector<ReferenceSuccessorCandidate> generate_reference_successor_candidates(
        Node *current)
    {
        std::vector<ReferenceSuccessorCandidate> candidates;
        const int nearest_idx = nearest_reference_index(current);
        if (nearest_idx < 0)
            return candidates;

        const int lookahead =
            std::max(1, reference_egraph.options.successor_lookahead);

        const double maxc = 1.0 / std::max(min_turn_radius, 1e-6);
        auto append_candidate = [&](int ref_idx, bool is_snap)
        {
            if (ref_idx < 0 ||
                ref_idx >= static_cast<int>(reference_egraph.nodes.size()))
                return;

            const Pose &target = reference_egraph.nodes[ref_idx].pose;
            const double distance =
                std::hypot(target.x - current->x, target.y - current->y);
            if (yaw_distance(target.yaw, current->yaw) >
                std::max(0.0, reference_egraph.options.snap_yaw))
                return;
            if (distance < 1e-6 &&
                yaw_distance(target.yaw, current->yaw) < 1e-6)
                return;

            auto paths = ReedShepp::calc_paths(current->x, current->y,
                                               current->yaw,
                                               target.x, target.y,
                                               target.yaw,
                                               maxc,
                                               params.rs_step_size,
                                               wheel_base);
            if (paths.empty())
                return;
            paths.erase(std::remove_if(paths.begin(), paths.end(),
                                       [&](const ReedShepp::Path &path)
                                       {
                                           return !analytic_path_allowed_for_mode(path);
                                       }),
                        paths.end());
            if (paths.empty())
                return;
            std::sort(paths.begin(), paths.end(),
                      [](const ReedShepp::Path &a, const ReedShepp::Path &b)
                      { return std::abs(a.L) < std::abs(b.L); });

            const ReedShepp::Path &path = paths.front();
            const double t = rs_arrival_time(current, path);
            if (t > params.max_time)
                return;

            const double edge_cost =
                std::abs(path.L) +
                0.1 * reference_transition_cost(
                          Pose(current->x, current->y, current->yaw),
                          target);
            Node candidate(target.x, target.y, target.yaw, t,
                           current->cost + edge_cost, 0.0, current, 1);
            ReferenceSuccessorCandidate ref_candidate;
            ref_candidate.node = candidate;
            ref_candidate.is_snap = is_snap;
            ref_candidate.local_path = path;
            candidates.push_back(std::move(ref_candidate));
        };

        append_candidate(nearest_idx, true);
        for (int step = 1; step <= lookahead; ++step)
            append_candidate(nearest_idx + step, false);

        return candidates;
    }

    std::vector<Waypoint> extract_path(Node *node, const std::vector<std::tuple<double, double, double>> &rs_path)
    {
        std::vector<Waypoint> waypoints = extract_parent_chain_path(node);
        if (!rs_path.empty())
        {
            double rs_t = waypoints.back().time;
            for (size_t i = 1; i < rs_path.size(); ++i)
            {
                auto [prev_x, prev_y, prev_yaw] = rs_path[i - 1];
                auto [x, y, yaw] = rs_path[i];
                double dist = std::hypot(x - prev_x, y - prev_y);
                rs_t += dist / speed;
                Waypoint w;
                w.time = rs_t;
                w.x = x;
                w.y = y;
                w.yaw = mod2pi(yaw);
                w.linear_velocity = speed;
                w.steering_angle = 0.0;
                append_waypoint_if_distinct(waypoints, w);
            }
        }
        return waypoints;
    }

    std::vector<Waypoint> extract_path(Node *node, const ReedShepp::Path &rs_path)
    {
        std::vector<Waypoint> waypoints = extract_parent_chain_path(node);

        double rs_t = waypoints.empty() ? node->t : waypoints.back().time;
        for (size_t i = 1; i < rs_path.x.size(); ++i)
        {
            const double dist = std::hypot(rs_path.x[i] - rs_path.x[i - 1],
                                           rs_path.y[i] - rs_path.y[i - 1]);
            rs_t += dist / speed;

            Waypoint w;
            w.time = rs_t;
            w.x = rs_path.x[i];
            w.y = rs_path.y[i];
            w.yaw = mod2pi(rs_path.yaw[i]);
            const int direction = (i < rs_path.directions.size()) ? rs_path.directions[i] : 1;
            w.linear_velocity = direction * speed;
            w.steering_angle = (i < rs_path.steers.size()) ? rs_path.steers[i] : 0.0;
            append_waypoint_if_distinct(waypoints, w);
        }

        return waypoints;
    }

public:
    // robot, goal, timetable, entities, params, is_transfer, obj_name, start_time
    PHAStar(RobotMeta *r, const Pose &goal_pose, TimeTable *tt, const std::unordered_map<std::string, EntityMeta *> *ents,
            const Params &p, bool trans = false, const std::string &obj_name = "", double start_t = 0.0,
            const std::string &debug_kind = "", const std::string &terminal_contact_name = "")
        : robot(r), timetable(tt), entities(ents), is_transfer(trans), debug_plan_kind(debug_kind), params(p)
    {
        // Update initial pose if chaining (but for now, assume caller updates r->initial_pose if needed)
        start = std::make_unique<Node>(r->initial_pose.x, r->initial_pose.y, mod2pi(r->initial_pose.yaw), start_t, 0.0, 0.0, nullptr, 1);
        goal = std::make_unique<Node>(goal_pose.x, goal_pose.y, mod2pi(goal_pose.yaw), 0.0, 0.0, 0.0, nullptr, 1);

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
        if (!terminal_contact_name.empty())
        {
            auto terminal_it = entities->find(terminal_contact_name);
            if (terminal_it != entities->end())
            {
                terminal_contact_entity = terminal_it->second;
            }
        }

        wheel_base = robot->wheel_base;
        min_turn_radius =
            std::max(1e-6, robot->turning_radius_for_mode(is_transfer));
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

    void set_reference_experience_graph(
        const std::vector<Waypoint> &reference_waypoints,
        const ReferenceExperienceGraphOptions &options)
    {
        build_reference_egraph(reference_waypoints, options);
    }

    bool has_reference_experience_graph() const
    {
        return reference_egraph.enabled;
    }

    // Check if a pose collides with any entity at given time (returns colliding entity name or "")
    std::string check_pose_collision(const Pose &pose, double check_time, bool include_transferred = false)
    {
        Corners pose_bounds_corners = get_corners(
            pose.x, pose.y, pose.yaw,
            robot->size.front_length, robot->size.rear_length, robot->size.width);

        // Check robot bounds using the configured boundary policy. Entity
        // collision checks below still use the full collision geometry.
        if (check_robot_bounds_collision(pose, pose_bounds_corners, params))
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
            if (ignore_other_robots && ent->type == EntityType::ROBOT)
                continue;
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

    bool is_terminal_approach_contact_pose(const Pose &robot_pose,
                                           EntityMeta *entity,
                                           const Pose &entity_pose) const
    {
        if (is_transfer || !terminal_contact_entity ||
            entity != terminal_contact_entity || !entity ||
            entity->type != EntityType::OBJECT)
        {
            return false;
        }

        const double goal_dist =
            std::hypot(robot_pose.x - goal->x, robot_pose.y - goal->y);
        const double goal_yaw =
            std::abs(pi_2_pi(robot_pose.yaw - goal->yaw));
        const double dist_tol = std::max(0.05, params.xy_resolution);
        const double yaw_tol = std::max(0.20, params.yaw_resolution);
        if (goal_dist > dist_tol || goal_yaw > yaw_tol)
        {
            return false;
        }

        return is_valid_transfer_contact_pose(
            robot_pose, dynamic_cast<ObjectMeta *>(entity), entity_pose);
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
        if (check_robot_bounds_collision(goal_pose, goal_corners, params))
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
                auto poses = timetable->get_poses(start_t);
                auto pose_it = poses.find(coll_ent);
                if (pose_it != poses.end() &&
                    is_terminal_approach_contact_pose(goal_pose, coll_ent,
                                                      pose_it->second))
                {
                    res.status = PlanningStatus::SUCCESS;
                    return res;
                }
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
        PlanningDebugStats stats;
        stats.planner_expansion_threads = std::max(1, planner_expansion_threads);
        stats.reference_egraph_nodes = reference_egraph.enabled
                                           ? reference_egraph.nodes.size()
                                           : 0;
        PHAStarExpansionWorkerPool expansion_pool(stats.planner_expansion_threads);
        using PlannerClock = std::chrono::steady_clock;
        auto add_elapsed = [](double &target, const PlannerClock::time_point &start_time)
        {
            target += std::chrono::duration<double>(
                          PlannerClock::now() - start_time)
                          .count();
        };
        auto is_soft_robot_block = [](const CollisionInfo &info)
        {
            return (info.reason.find("Robot") != std::string::npos) &&
                   (info.entity_name != "Boundary");
        };

        auto stamp_stats = [&](PlanningResult &out)
        {
            out.debug_stats = stats;
        };

        auto make_failure = [&](PlanningStatus status, const std::string &detail) -> PlanningResult
        {
            PlanningResult fail_res;
            fail_res.status = status;
            fail_res.failure_detail = detail;
            stats.final_failure_reason = detail;
            stamp_stats(fail_res);
            return fail_res;
        };

        auto update_best = [&](const Node *node)
        {
            const double dist = std::hypot(node->x - goal->x, node->y - goal->y);
            const double yaw_error = std::fabs(pi_2_pi(node->yaw - goal->yaw));
            if (dist < stats.best_dist ||
                (std::abs(dist - stats.best_dist) < 1e-9 &&
                 yaw_error < stats.best_yaw_error))
            {
                stats.best_dist = dist;
                stats.best_yaw_error = yaw_error;
                stats.best_pose = Pose(node->x, node->y, node->yaw);
            }
        };

        auto check_post_arrival = [&](double arrival_t) -> CollisionInfo
        {
            const double last_timetable_time = timetable->get_max_time();
            const double step = std::max(1e-3, params.collision_check_time_step);
            for (double future_t = arrival_t + step;
                 future_t <= last_timetable_time + 1e-9;
                 future_t += step)
            {
                Node dummy(goal->x, goal->y, goal->yaw,
                           future_t, 0.0, 0.0, nullptr, 0);
                auto collision_result = check_collision_at(&dummy);
                if (!collision_result.is_valid)
                {
                    return collision_result;
                }
            }
            return {true, "Valid", "", 0.0};
        };

        // Step 1: Quick start/goal validation
        PlanningResult validation = validate_start_goal(check_time, Pose(goal->x, goal->y, goal->yaw));
        if (validation.status != PlanningStatus::SUCCESS)
        {
            stats.final_failure_reason = validation.failure_detail;
            stamp_stats(validation);
            return validation;
        }

        std::vector<Node> local_explored; // for debug

        using PQElem = std::tuple<double, uint64_t, Node *>;
        auto cmp = [](const PQElem &a, const PQElem &b)
        { return std::get<0>(a) > std::get<0>(b) || (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) > std::get<1>(b)); };
        std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> anchor_open_set(cmp);
        std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> reference_open_set(cmp);
        const bool use_reference_mha = reference_egraph.enabled && !is_transfer;
        const double mha_selection_inflation =
            std::max(1.5, reference_egraph.options.epsilon);
        const double reference_aux_radius =
            std::max(reference_egraph.options.snap_radius * 2.0,
                     params.xy_resolution * 3.0);
        const double reference_aux_yaw =
            std::max(reference_egraph.options.snap_yaw,
                     params.yaw_resolution * 2.0);
        uint64_t node_id = 0;
        size_t start_id = calc_grid_index(start.get());
        std::unordered_map<size_t, double> g_costs;
        g_costs[start_id] = 0.0;
        auto open_size = [&]() -> std::size_t
        {
            return anchor_open_set.size() + reference_open_set.size();
        };
        auto is_reference_aux_relevant = [&](const Node *node) -> bool
        {
            return use_reference_mha &&
                   nearest_reference_index(node, reference_aux_radius,
                                           reference_aux_yaw) >= 0;
        };
        auto enqueue_reference_aux = [&](Node *node, bool force_reference)
        {
            if (!use_reference_mha)
                return;
            if (!force_reference && !is_reference_aux_relevant(node))
            {
                stats.mha_reference_skipped_far++;
                return;
            }

            auto heuristic_start = PlannerClock::now();
            double reference_h = calc_reference_egraph_heuristic(node);
            add_elapsed(stats.heuristic_time_sec, heuristic_start);
            if (!std::isfinite(reference_h))
                return;

            reference_open_set.emplace(node->cost + reference_h,
                                       node_id++, node);
            stats.mha_reference_queued++;
        };
        auto enqueue_node = [&](Node *node,
                                double anchor_heuristic,
                                bool force_reference = false)
        {
            anchor_open_set.emplace(node->cost + anchor_heuristic,
                                    node_id++, node);
            enqueue_reference_aux(node, force_reference);
            stats.peak_open_size =
                std::max<std::size_t>(stats.peak_open_size, open_size());
        };

        enqueue_node(start.get(), calc_anchor_heuristic(start.get()), true);
        stats.accepted_nodes = 1;
        std::unordered_set<size_t> closed_set;

        auto discard_stale_entries =
            [&](std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> &queue)
        {
            while (!queue.empty())
            {
                Node *node = std::get<2>(queue.top());
                const size_t id = calc_grid_index(node);
                auto g_it = g_costs.find(id);
                if (closed_set.count(id) ||
                    g_it == g_costs.end() ||
                    node->cost > g_it->second)
                {
                    queue.pop();
                    continue;
                }
                break;
            }
        };

        size_t iteration = 0; // for debug

        // Clear previous backup
        backup_result.status = PlanningStatus::INTERNAL_ERROR;
        backup_result.waypoints.clear();

        while (!anchor_open_set.empty() ||
               (use_reference_mha && !reference_open_set.empty()))
        {
            discard_stale_entries(anchor_open_set);
            if (use_reference_mha)
                discard_stale_entries(reference_open_set);
            if (anchor_open_set.empty() &&
                (!use_reference_mha || reference_open_set.empty()))
                break;

            iteration++;
            stats.iterations = iteration;
            if (iteration % 1000 == 0)
            {
                const double anchor_key = anchor_open_set.empty()
                                              ? std::numeric_limits<double>::infinity()
                                              : std::get<0>(anchor_open_set.top());
                const double reference_key =
                    (!use_reference_mha || reference_open_set.empty())
                        ? std::numeric_limits<double>::infinity()
                        : std::get<0>(reference_open_set.top());
                std::cout << "A* iteration: " << iteration
                          << ", anchor_open size: " << anchor_open_set.size()
                          << ", reference_open size: " << reference_open_set.size()
                          << ", anchor f-cost: " << anchor_key
                          << ", reference f-cost: " << reference_key
                          << std::endl;
            }

            // Early exit if search is taking too long
            if (iteration > (size_t)max_search_iterations)
            {
                if (backup_result.status == PlanningStatus::BLOCKED_BY_ROBOT)
                {
                    std::cout << "Hit iteration limit (" << max_search_iterations << ") with backup path. Returning backup." << std::endl;
                    stats.final_failure_reason = backup_result.failure_detail;
                    stamp_stats(backup_result);
                    return backup_result;
                }
                else
                {
                    std::cout << "Hit iteration limit (" << max_search_iterations << ") with NO backup. Aborting." << std::endl;
                    return make_failure(PlanningStatus::NO_PATH_FOUND,
                                        "Search iteration limit exceeded");
                }
            }

            bool expand_reference_queue = false;
            if (use_reference_mha && !reference_open_set.empty())
            {
                expand_reference_queue =
                    anchor_open_set.empty() ||
                    std::get<0>(reference_open_set.top()) <=
                        mha_selection_inflation *
                            std::get<0>(anchor_open_set.top());
            }

            PQElem open_entry =
                expand_reference_queue ? reference_open_set.top()
                                       : anchor_open_set.top();
            Node *current = std::get<2>(open_entry);
            if (expand_reference_queue)
            {
                reference_open_set.pop();
                stats.mha_reference_expansions++;
            }
            else
            {
                anchor_open_set.pop();
                stats.mha_anchor_expansions++;
            }
            // for debug
            local_explored.push_back(*current);

            // for debug
            // std::cout << "DEBUG_VISITED," << current->x << "," << current->y << ","
            //         << current->yaw << "," << current->t << "," << current->cost << std::endl;

            size_t n_id = calc_grid_index(current);
            if (closed_set.count(n_id))
            {
                stats.reject_closed++;
                continue;
            }
            if (current->cost > g_costs[n_id])
            {
                stats.reject_worse_g++;
                continue; // Outdated entry
            }
            closed_set.insert(n_id);
            stats.closed_nodes = closed_set.size();
            update_best(current);

            // for debug
            if (iteration % 5000 == 0)
            {
                std::cout << "  Current node: t=" << current->t << ", x=" << current->x << ", y=" << current->y << ", yaw=" << current->yaw
                          << ", cost=" << current->cost << std::endl;
            }

            auto is_collision_free = check_collision_at(current);
            if (!is_collision_free.is_valid)
            {
                stats.reject_collision++;
                continue;
            }

            struct AnalyticCandidateEval
            {
                CollisionInfo rs_info;
                CollisionInfo post_arrival_info;
                double arrival_t = 0.0;
            };

            auto evaluate_analytic_candidate =
                [&](const ReedShepp::Path &rs_candidate) -> AnalyticCandidateEval
            {
                AnalyticCandidateEval eval;
                auto rs_path = rs_points(rs_candidate);
                eval.rs_info = check_collision_along_rs(rs_path, current->t);
                if (eval.rs_info.is_valid)
                {
                    eval.arrival_t = rs_arrival_time(current, rs_candidate);
                    eval.post_arrival_info = check_post_arrival(eval.arrival_t);
                }
                return eval;
            };

            auto handle_analytic_candidate =
                [&](const ReedShepp::Path &rs_candidate,
                    const AnalyticCandidateEval &eval) -> PlanningResult
            {
                PlanningResult no_result;
                CollisionInfo rs_info = eval.rs_info;
                if (rs_info.is_valid)
                {
                    auto waypoints = extract_path(current, rs_candidate);
                    CollisionInfo post_arrival_collision =
                        eval.post_arrival_info;

                    if (post_arrival_collision.is_valid)
                    {
                        std::cout << "Goal found with analytic expansion!" << std::endl;
                        PlanningResult success;
                        success.waypoints = waypoints;
                        success.status = PlanningStatus::SUCCESS;
                        stamp_stats(success);
                        return success;
                    }

                    stats.analytic_post_arrival_collision++;
                    if (is_soft_robot_block(post_arrival_collision))
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
                    stats.analytic_collision++;
                    if (is_soft_robot_block(rs_info))
                    {
                        // This path is blocked by a robot, but geometerically valid.
                        if (backup_result.status != PlanningStatus::BLOCKED_BY_ROBOT)
                        {
                            backup_result.waypoints = extract_path(current, rs_candidate);
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
                return no_result;
            };

            auto rs_candidates = analytic_expand_all(current);
            if (!rs_candidates.empty())
            {
                if (expansion_pool.parallel_enabled(rs_candidates.size()))
                {
                    std::vector<AnalyticCandidateEval> analytic_evals(rs_candidates.size());
                    auto eval_start = PlannerClock::now();
                    expansion_pool.parallel_for(
                        rs_candidates.size(),
                        [&](std::size_t idx)
                        {
                            analytic_evals[idx] =
                                evaluate_analytic_candidate(rs_candidates[idx]);
                        });
                    add_elapsed(stats.analytic_validation_time_sec, eval_start);

                    auto merge_start = PlannerClock::now();
                    for (std::size_t idx = 0; idx < rs_candidates.size(); ++idx)
                    {
                        PlanningResult analytic_result =
                            handle_analytic_candidate(rs_candidates[idx],
                                                      analytic_evals[idx]);
                        if (analytic_result.status == PlanningStatus::SUCCESS)
                        {
                            add_elapsed(stats.serial_merge_time_sec, merge_start);
                            analytic_result.debug_stats = stats;
                            return analytic_result;
                        }
                    }
                    add_elapsed(stats.serial_merge_time_sec, merge_start);
                }
                else
                {
                    for (const auto &rs_candidate : rs_candidates)
                    {
                        auto eval_start = PlannerClock::now();
                        AnalyticCandidateEval eval =
                            evaluate_analytic_candidate(rs_candidate);
                        add_elapsed(stats.analytic_validation_time_sec, eval_start);

                        auto merge_start = PlannerClock::now();
                        PlanningResult analytic_result =
                            handle_analytic_candidate(rs_candidate, eval);
                        add_elapsed(stats.serial_merge_time_sec, merge_start);
                        if (analytic_result.status == PlanningStatus::SUCCESS)
                        {
                            analytic_result.debug_stats = stats;
                            return analytic_result;
                        }
                    }
                }
            }

            // Goal check (assuming near-goal threshold)
            double dist_to_goal = std::hypot(current->x - goal->x, current->y - goal->y);
            double dyaw_to_goal = std::fabs(pi_2_pi(current->yaw - goal->yaw));
            if (dist_to_goal < params.xy_resolution && dyaw_to_goal < params.yaw_resolution)
            {
                auto waypoints =
                    extract_path(current,
                                 std::vector<std::tuple<double, double, double>>{});
                double arrival_t = waypoints.back().time;
                CollisionInfo post_arrival_collision =
                    check_post_arrival(arrival_t);

                if (post_arrival_collision.is_valid)
                {
                    std::cout << "Goal found!" << std::endl;
                    // return {waypoints, PlanningStatus::SUCCESS, ""};
                    res.waypoints = waypoints;
                    res.status = PlanningStatus::SUCCESS;
                    stamp_stats(res);
                    return res;
                }
                stats.analytic_post_arrival_collision++;
                // Else continue
            }

            struct PrimitiveCandidateEval
            {
                bool generated = false;
                bool collision_free = false;
                Node node;
                double heuristic = std::numeric_limits<double>::infinity();
                double collision_time_sec = 0.0;
                double heuristic_time_sec = 0.0;
            };

            auto evaluate_primitive_candidate =
                [&](const std::pair<int, double> &prim) -> PrimitiveCandidateEval
            {
                PrimitiveCandidateEval eval;
                eval.generated = compute_generated_node(current, prim, eval.node);
                if (!eval.generated)
                    return eval;

                auto collision_start = PlannerClock::now();
                eval.collision_free =
                    check_collision_along_path(current, &eval.node, prim);
                add_elapsed(eval.collision_time_sec, collision_start);
                if (!eval.collision_free)
                    return eval;

                auto heuristic_start = PlannerClock::now();
                eval.heuristic = calc_anchor_heuristic(&eval.node);
                add_elapsed(eval.heuristic_time_sec, heuristic_start);
                return eval;
            };

            auto merge_primitive_candidate =
                [&](const PrimitiveCandidateEval &eval)
            {
                if (!eval.generated)
                    return;
                stats.generated_nodes++;
                stats.primitive_collision_time_sec += eval.collision_time_sec;
                stats.heuristic_time_sec += eval.heuristic_time_sec;

                if (!eval.collision_free)
                {
                    stats.reject_collision++;
                    return;
                }

                size_t new_id = calc_grid_index(&eval.node);
                if (closed_set.count(new_id))
                {
                    stats.reject_closed++;
                    return;
                }
                double new_g = eval.node.cost;
                auto it = g_costs.find(new_id);
                if (it != g_costs.end() && new_g >= it->second)
                {
                    stats.reject_worse_g++;
                    return; // Worse or equal
                }
                g_costs[new_id] = new_g;
                Node *accepted_node = this->new_node(
                    eval.node.x, eval.node.y, eval.node.yaw, eval.node.t,
                    eval.node.cost, eval.node.steer, current,
                    eval.node.direction);
                enqueue_node(accepted_node, eval.heuristic);
                stats.accepted_nodes++;
            };

            auto merge_reference_successors = [&]()
            {
                if (!reference_egraph.enabled)
                    return;

                auto ref_candidates =
                    generate_reference_successor_candidates(current);
                for (const auto &ref_candidate : ref_candidates)
                {
                    const Node &candidate = ref_candidate.node;
                    const bool is_snap = ref_candidate.is_snap;
                    stats.generated_nodes++;
                    auto collision_start = PlannerClock::now();
                    CollisionInfo rs_info = check_collision_along_rs(
                        rs_points(ref_candidate.local_path), current->t);
                    add_elapsed(stats.primitive_collision_time_sec,
                                collision_start);
                    if (!rs_info.is_valid)
                    {
                        stats.reject_collision++;
                        if (is_snap)
                            stats.reference_egraph_snap_rejected++;
                        else
                            stats.reference_egraph_successor_rejected++;
                        continue;
                    }

                    size_t new_id = calc_grid_index(&candidate);
                    if (closed_set.count(new_id))
                    {
                        stats.reject_closed++;
                        if (is_snap)
                            stats.reference_egraph_snap_rejected++;
                        else
                            stats.reference_egraph_successor_rejected++;
                        continue;
                    }

                    double new_g = candidate.cost;
                    auto it = g_costs.find(new_id);
                    if (it != g_costs.end() && new_g >= it->second)
                    {
                        stats.reject_worse_g++;
                        if (is_snap)
                            stats.reference_egraph_snap_rejected++;
                        else
                            stats.reference_egraph_successor_rejected++;
                        continue;
                    }

                    g_costs[new_id] = new_g;
                    auto heuristic_start = PlannerClock::now();
                    double heuristic = calc_anchor_heuristic(
                        const_cast<Node *>(&candidate));
                    add_elapsed(stats.heuristic_time_sec, heuristic_start);

                    Node *chain_parent = current;
                    double chain_time = current->t;
                    double chain_cost = current->cost;
                    for (std::size_t i = 1;
                         i < ref_candidate.local_path.x.size(); ++i)
                    {
                        const double dist = std::hypot(
                            ref_candidate.local_path.x[i] -
                                ref_candidate.local_path.x[i - 1],
                            ref_candidate.local_path.y[i] -
                                ref_candidate.local_path.y[i - 1]);
                        chain_time += dist / std::max(speed, 1e-6);
                        const int direction =
                            (i < ref_candidate.local_path.directions.size())
                                ? ref_candidate.local_path.directions[i]
                                : 1;
                        const double steer =
                            (i < ref_candidate.local_path.steers.size())
                                ? ref_candidate.local_path.steers[i]
                                : 0.0;
                        chain_cost += std::abs(dist);

                        chain_parent = this->new_node(
                            ref_candidate.local_path.x[i],
                            ref_candidate.local_path.y[i],
                            mod2pi(ref_candidate.local_path.yaw[i]),
                            chain_time,
                            chain_cost,
                            steer,
                            chain_parent,
                            direction);
                    }

                    if (!chain_parent || chain_parent == current)
                        continue;
                    chain_parent->cost = candidate.cost;
                    chain_parent->t = candidate.t;
                    enqueue_node(chain_parent, heuristic, true);
                    stats.accepted_nodes++;
                    if (is_snap)
                        stats.reference_egraph_snap_accepted++;
                    else
                        stats.reference_egraph_successor_accepted++;
                }
            };

            if (expansion_pool.parallel_enabled(motion_primitives.size()))
            {
                std::vector<PrimitiveCandidateEval> primitive_evals(motion_primitives.size());
                expansion_pool.parallel_for(
                    motion_primitives.size(),
                    [&](std::size_t idx)
                    {
                        primitive_evals[idx] =
                            evaluate_primitive_candidate(motion_primitives[idx]);
                    });

                auto merge_start = PlannerClock::now();
                for (const auto &eval : primitive_evals)
                    merge_primitive_candidate(eval);
                add_elapsed(stats.serial_merge_time_sec, merge_start);
            }
            else
            {
                for (auto prim : motion_primitives)
                {
                    Node *new_node = generate_node(current, prim);
                    if (!new_node)
                        continue;
                    stats.generated_nodes++;
                    auto collision_start = PlannerClock::now();
                    bool collision_free =
                        check_collision_along_path(current, new_node, prim);
                    add_elapsed(stats.primitive_collision_time_sec, collision_start);
                    if (!collision_free)
                    {
                        stats.reject_collision++;
                        continue;
                    }

                    auto merge_start = PlannerClock::now();
                    size_t new_id = calc_grid_index(new_node);
                    if (closed_set.count(new_id))
                    {
                        stats.reject_closed++;
                        add_elapsed(stats.serial_merge_time_sec, merge_start);
                        continue;
                    }
                    double new_g = new_node->cost;
                    auto it = g_costs.find(new_id);
                    if (it != g_costs.end() && new_g >= it->second)
                    {
                        stats.reject_worse_g++;
                        add_elapsed(stats.serial_merge_time_sec, merge_start);
                        continue; // Worse or equal
                    }
                    g_costs[new_id] = new_g;
                    add_elapsed(stats.serial_merge_time_sec, merge_start);

                    auto heuristic_start = PlannerClock::now();
                    double heuristic = calc_anchor_heuristic(new_node);
                    add_elapsed(stats.heuristic_time_sec, heuristic_start);
                    enqueue_node(new_node, heuristic);
                    stats.accepted_nodes++;
                }
            }

            auto reference_merge_start = PlannerClock::now();
            merge_reference_successors();
            add_elapsed(stats.serial_merge_time_sec, reference_merge_start);
        }

        if (backup_result.status == PlanningStatus::BLOCKED_BY_ROBOT)
        {
            std::cout << "Returning backup path blocked by " << backup_result.colliding_entity << std::endl;
            stats.final_failure_reason = backup_result.failure_detail;
            stamp_stats(backup_result);
            return backup_result;
        }

        std::cout << "No path found" << std::endl;
        res.status = PlanningStatus::NO_PATH_FOUND;
        res.failure_detail = "Search exhausted without reaching goal.";
        res.explored_nodes = std::move(local_explored); // Move to result on failure
        stats.final_failure_reason = res.failure_detail;
        stamp_stats(res);
        // return {};
        return res;
    }

    void set_ignore_other_robots(bool ignore)
    {
        ignore_other_robots = ignore;
    }

    void set_planner_expansion_threads(int threads)
    {
        planner_expansion_threads = std::max(1, threads);
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
                auto waypoints =
                    extract_path(current,
                                 std::vector<std::tuple<double, double, double>>{});
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
            traj.kind = trans ? TrajectoryKind::TRANSFER : TrajectoryKind::TRANSIT;
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
