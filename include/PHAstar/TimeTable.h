#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <iostream>
#include <iomanip>
#include <map>
#include <unordered_map>
#include <vector>

#include <PHAstar/Entities.h>
#include <PHAstar/Utils.h> // Added for mod2pi
#include <PHAstar/CollisionUtils.h>

class TimeTable
{
public:
    struct TrajectorySpan
    {
        EntityMeta *entity = nullptr;
        EntityMeta *transferred_object = nullptr;
        double start_time = 0.0;
        double end_time = 0.0;
        bool is_transfer = false;
        TrajectoryKind kind = TrajectoryKind::TRANSIT;
    };

    double time_increment = 0.5;

    TimeTable(double inc = 0.5) : time_increment(inc) {}

    void add_initial(const std::unordered_map<std::string, EntityMeta *> &entities)
    {
        for (const auto &[name, ent] : entities)
        {
            per_entity_table[ent][0.0] = ent->initial_pose;
        }
    }

    void add_trajectory(const Trajectory &traj)
    {
        EntityMeta *ent = traj.entity;
        if (!ent || traj.waypoints.empty())
            return;
        double min_relative = traj.waypoints.front().time;
        double max_relative = traj.waypoints.back().time;
        double offset = traj.start_time;
        double absolute_min_t = offset + min_relative;
        double absolute_max_t = offset + max_relative;
        TrajectoryKind effective_kind =
            traj.is_transfer ? TrajectoryKind::TRANSFER : traj.kind;
        trajectory_spans.push_back({ent, traj.transferred_object,
                                    absolute_min_t, absolute_max_t,
                                    traj.is_transfer, effective_kind});
        double initial_obj_yaw = 0.0;
        double robot_start_yaw = traj.waypoints.front().yaw;
        if (traj.is_transfer && traj.transferred_object)
        {
            initial_obj_yaw = get_pose(traj.transferred_object, absolute_min_t).yaw;
        }

        constexpr double kHoldSampleEps = 1e-4;
        auto &entity_table = per_entity_table[ent];
        auto next_it = entity_table.lower_bound(absolute_min_t);
        if (next_it != entity_table.begin())
        {
            auto prev_it = std::prev(next_it);
            if ((absolute_min_t - prev_it->first) > kHoldSampleEps)
            {
                const double hold_time = absolute_min_t - kHoldSampleEps;
                entity_table[hold_time] = prev_it->second;
                if (traj.is_transfer && traj.transferred_object)
                {
                    per_entity_table[traj.transferred_object][hold_time] =
                        get_pose(traj.transferred_object, hold_time);
                }
            }
        }

        auto record_robot_pose = [&](double absolute_t, const Pose &p)
        {
            per_entity_table[ent][absolute_t] = p;
            if (traj.is_transfer && traj.transferred_object)
            {
                Pose obj_p = compute_object_pose(p, ent->size, traj.transferred_object->size);
                double delta_yaw = mod2pi(p.yaw - robot_start_yaw);
                obj_p.yaw = mod2pi(initial_obj_yaw + delta_yaw);
                per_entity_table[traj.transferred_object][absolute_t] = obj_p;
            }
        };

        // Store the exact trajectory waypoints so later interpolation follows
        // the same path that was collision-checked during scheduling.
        for (const auto &wp : traj.waypoints)
        {
            record_robot_pose(offset + wp.time, wp);
        }

        // Also keep intermediate timetable samples for queries between waypoints.
        for (double absolute_t = absolute_min_t; absolute_t <= absolute_max_t + 1e-9; absolute_t += time_increment)
        {
            double relative_t = absolute_t - offset;
            if (relative_t >= min_relative && relative_t <= max_relative)
            {
                Pose p = interpolate_waypoints(traj.waypoints, relative_t);
                record_robot_pose(absolute_t, p);
            }
        }
    }

    Pose get_pose(EntityMeta *ent, double t) const
    {
        if (!ent)
        {
            return {};
        }

        auto it = per_entity_table.find(ent);
        if (it == per_entity_table.end() || it->second.empty())
        {
            return ent->initial_pose;
        }
        const auto &m = it->second;
        auto it_upper = m.upper_bound(t);
        if (it_upper == m.begin())
        {
            return m.begin()->second;
        }
        if (it_upper == m.end())
        {
            auto it_last = m.end();
            --it_last;
            return it_last->second;
        }
        auto it_lower = it_upper;
        --it_lower;
        if (it_lower->first == t)
        {
            return it_lower->second;
        }

        // Object poses are piecewise-constant in the timetable: they stay at the
        // last known pose until the next explicitly recorded pose exists.
        if (ent->type == EntityType::OBJECT)
        {
            return it_lower->second;
        }

        return interpolate_pose(it_lower->second, it_lower->first, it_upper->second, it_upper->first, t);
    }

    std::unordered_map<EntityMeta *, Pose> get_poses(double t) const
    {
        std::unordered_map<EntityMeta *, Pose> poses;
        for (const auto &[ent, m] : per_entity_table)
        {
            poses[ent] = get_pose(ent, t);
        }
        return poses;
    }

    // for visualization
    const std::unordered_map<EntityMeta *, std::map<double, Pose>> &get_database() const
    {
        return per_entity_table;
    }

    const std::vector<TrajectorySpan> &get_trajectory_spans() const
    {
        return trajectory_spans;
    }

    double get_max_time() const
    {
        double max_t = 0.0;
        for (const auto &[ent, m] : per_entity_table)
        {
            if (!m.empty())
            {
                max_t = std::max(max_t, m.rbegin()->first);
            }
        }
        return max_t;
    }

    double get_entity_max_time(EntityMeta *ent, double margin = 0.5) const
    {
        auto it = per_entity_table.find(ent);
        if (it == per_entity_table.end() || it->second.empty())
        {
            return 0.0;
        }
        return it->second.rbegin()->first + margin;
    }

    // In TimeTable class (TimeTable.h)
    bool is_entity_static_after(double after_t, EntityMeta *ent) const
    {
        auto it = per_entity_table.find(ent);
        if (it == per_entity_table.end())
            return true; // No trajectory → static

        const auto &time_pose_map = it->second;
        if (time_pose_map.empty())
            return true;

        // Find the latest time >= after_t
        auto latest_it = time_pose_map.lower_bound(after_t);
        if (latest_it == time_pose_map.end())
        {
            // No future poses; check if last pose is constant (assume yes if no more entries)
            return true;
        }

        // If multiple poses after after_t, but all identical → static
        Pose last_pose = latest_it->second;
        ++latest_it;
        for (; latest_it != time_pose_map.end(); ++latest_it)
        {
            if (std::hypot(last_pose.x - latest_it->second.x, last_pose.y - latest_it->second.y) > 1e-3 ||
                std::abs(mod2pi(last_pose.yaw - latest_it->second.yaw)) > 1e-3)
            {
                return false; // Still moving
            }
        }
        return true; // No changes → static
    }

    static Pose compute_object_pose(const Pose &robot_pose, const OccuRect &robot_size, const OccuRect &obj_size)
    {
        Pose obj_pose;
        double offset = robot_size.front_length + obj_size.rear_length;
        obj_pose.x = robot_pose.x + offset * std::cos(robot_pose.yaw);
        obj_pose.y = robot_pose.y + offset * std::sin(robot_pose.yaw);
        obj_pose.yaw = robot_pose.yaw;
        return obj_pose;
    }

    static Pose interpolate_waypoints(const std::vector<Waypoint> &waypoints, double t)
    {
        if (waypoints.empty())
        {
            return {};
        }
        if (t <= waypoints.front().time)
            return waypoints.front();
        if (t >= waypoints.back().time)
            return waypoints.back();
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            if (waypoints[i].time <= t && t <= waypoints[i + 1].time)
            {
                return interpolate_pose(waypoints[i], waypoints[i].time, waypoints[i + 1], waypoints[i + 1].time, t);
            }
        }
        return waypoints.back();
    }

    // Check if an entity is waiting (idle) at time t.
    // "Waiting" is defined as: query time t is >= the end time of the last registered plan.
    bool is_waiting(EntityMeta *ent, double time) const
    {
        auto it = per_entity_table.find(ent);
        if (it == per_entity_table.end())
            return true; // No plan means waiting

        const auto &plan = it->second;
        if (plan.empty())
            return true;

        // Get the timestamp of the very last pose registered
        double last_time = plan.rbegin()->first;

        // If queried time is past the robot's last move, it is waiting
        return time >= last_time;
    }

    // Find if any OTHER robot is blocking the target_pose at time t.
    // Returns the pointer to the blocking entity if found, otherwise nullptr.
    RobotMeta *get_blocking_robot(const Pose &target_pose, double t, TimeTable &timetable,
                                  double margin_m = 0.1)
    {
        // Setup geometry for the target pose (assuming standard robot dimensions)
        OccuRect assumed_size;
        assumed_size.front_length = 0.3;
        assumed_size.rear_length = 0.2;
        assumed_size.width = 0.3;

        // Apply margin as inflation
        double inflation = 1.0 + (margin_m / assumed_size.front_length);
        CollisionGeometry target_geom = setup_collision_geometry(target_pose, assumed_size, inflation);

        // Check against all robots in TimeTable
        auto others = timetable.get_poses(t);
        for (auto &[ent, ent_pose] : others)
        {
            if (ent->type != EntityType::ROBOT)
                continue; // Only check robots

            // Create a minimal Params for collision checking
            Params dummy_params;
            dummy_params.inflation = 1.0;
            dummy_params.safety_margin = 0.0;

            auto collision = check_entity_collision(target_geom, target_pose, ent, ent_pose, dummy_params);
            if (collision.has_collision)
            {
                return dynamic_cast<RobotMeta *>(ent);
            }
        }
        return nullptr;
    }

private:
    std::unordered_map<EntityMeta *, std::map<double, Pose>> per_entity_table;
    std::vector<TrajectorySpan> trajectory_spans;

    static Pose interpolate_pose(const Pose &p1, double t1, const Pose &p2, double t2, double t)
    {
        if (t1 == t2)
            return p1;
        double frac = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + frac * (p2.x - p1.x);
        p.y = p1.y + frac * (p2.y - p1.y);
        // double dyaw = mod2pi(p2.yaw - p1.yaw); // mod2pi causes funny spinning
        // p.yaw = mod2pi(p1.yaw + frac * dyaw);
        double dyaw = pi_2_pi(p2.yaw - p1.yaw);
        double yaw = p1.yaw + frac * dyaw;
        p.yaw = pi_2_pi(yaw);
        p.yaw = mod2pi(p.yaw);
        return p;
    }
};

void print_timetable_poses(const std::unordered_map<std::string, EntityMeta *> &entities, const std::vector<Trajectory> &all_trajectories, const TimeTable &timetable)
{
    std::cout << "TimeTable poses for robot2:" << std::endl;
    EntityMeta *robot2_ent = entities.at("robot2");
    double max_tt = 0.0;
    for (const auto &traj : all_trajectories)
    {
        if (!traj.waypoints.empty())
        {
            max_tt = std::max(max_tt, traj.waypoints.back().time);
        }
    }
    for (double t = 0.0; t <= max_tt + 1e-6; t += 0.1)
    { // Step of 0.1 for fine-grained view
        Pose p = timetable.get_pose(robot2_ent, t);
        std::cout << "t=" << std::fixed << std::setprecision(4) << t
                  << ", x=" << p.x << ", y=" << p.y << ", yaw=" << p.yaw << std::endl;
    }
}

#endif // TIMETABLE_H
