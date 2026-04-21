#ifndef ENTITIES_H
#define ENTITIES_H

#include <string>
#include <vector>
#include <memory.h>

#include <PHAstar/Point.h>

enum class EntityType { ROBOT, OBJECT };


struct OccuRect {
    double front_length = 0.0;
    double rear_length = 0.0;
    double width = 0.0;
};

struct EntityMeta {
    std::string name;
    EntityType type;
    Pose initial_pose;
    OccuRect size;
    virtual ~EntityMeta() = default;  // Added to make polymorphic
};

struct RobotMeta : public EntityMeta {
    double min_turning_radius = 0.0;
    double wheel_base = 0.0;
    double speed_transit = 0.0;
    double speed_transfer = 0.0;
};

struct ObjectMeta : public EntityMeta {
    Pose goal_pose;
};

struct Waypoint : public Pose {
    double time = 0.0;
    double linear_velocity = 0.0;
    double steering_angle = 0.0;

    Waypoint(){}

    Waypoint(Pose& p_in) : Pose(p_in)
    {}
};

typedef std::vector<Waypoint> WaypointPath;

/*
     * Robot, object, time_start, path, is_transfer
     */
struct Trajectory {
    EntityMeta* entity = nullptr;
    EntityMeta* transferred_object = nullptr;
    double start_time = 0.0;
    std::vector<Waypoint> waypoints;
    bool is_transfer = false;

    Trajectory(){}
    Trajectory(RobotMeta* robot_in, ObjectMeta* object_in,
               double time_start, WaypointPath path_in, bool is_transfer_in)
        : entity(robot_in), transferred_object(object_in), start_time(time_start),
            waypoints(path_in), is_transfer(is_transfer_in)
    {}

    // assign timestamp to each waypoints based on velocity
    void CalcualteTimeStamps(RobotMeta* robot, double start_time_in = 0.0)
    {
        if (waypoints.empty()) return;

        double speed = is_transfer ? robot->speed_transfer : robot->speed_transit;
        if (speed <= 0.0) return;  // Avoid division by zero

        double current_time = start_time_in;  // Or 0.0 if not set
        waypoints[0].time = 0; // it will be an increment from the pivot starting time of the trajectory

        for (size_t i = 1; i < waypoints.size(); ++i) {
            double dx = waypoints[i].x - waypoints[i-1].x;
            double dy = waypoints[i].y - waypoints[i-1].y;
            double distance = std::hypot(dx, dy);
            double delta_t = distance / speed;
            current_time += delta_t;
            waypoints[i].time = current_time;
        }
    }

};

typedef std::shared_ptr<Trajectory> TrajectoryPtr;


#endif // ENTITIES_H
