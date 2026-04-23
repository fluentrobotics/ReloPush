#ifndef TASK_H
#define TASK_H

#include <PHAstar/Entities.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/Utils.h>
#include <ReloPush.h>

inline double NormalizeReloPushYaw(double yaw_in)
{
    return mod2pi(yaw_in);
}

Pose PoseFromReloPushState(const ReloPush::State& state_in)
{
    Pose p;
    p.x = state_in.x;
    p.y = state_in.y;
    p.yaw = NormalizeReloPushYaw(state_in.yaw);

    return p;
}

Waypoint WaypointFromReloPushState(const ReloPush::State& state_in)
{
    Pose p = PoseFromReloPushState(state_in);

    Waypoint wp(p);
    wp.time = state_in.time;
    wp.linear_velocity = state_in.vel;

    return wp;
}

inline bool TryExtractEdgePathEndpointPose(const EdgePath& edge_path,
                                           bool use_front,
                                           Pose& out_pose)
{
    if (!std::holds_alternative<ReloPush::StatePathPtr>(edge_path.path)) {
        return false;
    }

    const auto& path_ptr = std::get<ReloPush::StatePathPtr>(edge_path.path);
    if (!path_ptr || path_ptr->empty()) {
        return false;
    }

    out_pose = PoseFromReloPushState(use_front ? path_ptr->front() : path_ptr->back());
    return true;
}

inline Pose DetermineTaskStartPoseRobot(const FinalAllocation& fa)
{
    Pose pose_out{};

    if (fa.obsReloPaths) {
        for (const auto& obs_path : *fa.obsReloPaths) {
            if (TryExtractEdgePathEndpointPose(obs_path, true, pose_out)) {
                return pose_out;
            }
        }
    }

    for (const auto& edge_group : fa.paths) {
        for (const auto& path_ptr : edge_group.paths) {
            if (path_ptr && TryExtractEdgePathEndpointPose(*path_ptr, true, pose_out)) {
                return pose_out;
            }
        }
    }

    if (fa.firstApproachPath && !fa.firstApproachPath->empty()) {
        return PoseFromReloPushState(fa.firstApproachPath->back());
    }

    return pose_out;
}

TrajectoryPtr ReloPushPath2TrajPtr(const std::shared_ptr<EdgePath> edgePath,
                                   RobotMeta* robot_in=nullptr, EntityMeta* transferred_obj=nullptr,
                                   double start_time = 0.0,
                                   double source_pre_push_distance = 0.0) {
    Trajectory traj;
    traj.entity = robot_in;
    traj.transferred_object = transferred_obj;
    traj.start_time = start_time;
    traj.is_transfer = edgePath->is_pushing;
    traj.source_pre_push_distance = source_pre_push_distance;

    if (!std::holds_alternative<ReloPush::StatePathPtr>(edgePath->path)) {
        return std::make_shared<Trajectory>(traj); // Empty if not StatePath
    }
    const auto& path_ptr = std::get<ReloPush::StatePathPtr>(edgePath->path);
    if (!path_ptr || path_ptr->empty()) {
        return std::make_shared<Trajectory>(traj);
    }
    const auto& states = *path_ptr;
    //traj.start_time = states[0].time;
    traj.start_time = -1; // time not assigned yet
    for (const auto& state : states) {
        Waypoint wp = WaypointFromReloPushState(state);
        wp.steering_angle = 0.0; // Not provided
        traj.waypoints.push_back(wp);
    }
    return std::make_shared<Trajectory>(traj);
}

TrajectoryPtr ReloPushPath2TrajPtr(const EdgePath edgePath,
                                   RobotMeta* robot_in=nullptr, EntityMeta* transferred_obj=nullptr,
                                   double start_time = 0.0,
                                   double source_pre_push_distance = 0.0)
{
    return ReloPushPath2TrajPtr(std::make_shared<EdgePath>(edgePath),robot_in,
                                transferred_obj,start_time,
                                source_pre_push_distance);
}

enum DependType { TRANSIT, TRANSFER };

class Task {
public:
    std::vector<Trajectory> transitTrajectoryRobot; // to be depricated
    std::vector<Trajectory> transferTrajectoryRobot; // to be depricated

    std::vector<TrajectoryPtr> EdgePaths;

    Pose transitGoal; // to be depricated
    Pose TaskStartPoseRobot; // starting pose of the task
    Pose StartPoseObj;
    Pose GoalPoseObj;
    std::pair<Task*, DependType> transferDepend;
    std::pair<Task*, DependType> transitDepend;
    RobotMeta* assignedRobot = nullptr;
    ObjectMeta* targetObject = nullptr;
    EntityMeta* initialApproachEntity = nullptr;
    double sourcePrePushDistance = 0.0;

    ////// added to handle obsRelo (need to verify)
    ReloPush::StatePathPtr firstApproachPath;
    std::shared_ptr<std::vector<EdgePath>> obsReloPaths;
    std::unordered_map<std::string, ReloPush::State> obsReloUpdate;
    std::vector<VertexData> vertexChain;
    ///////

    // constructor without assigned robot
    Task(const FinalAllocation& fa, const std::unordered_map<std::string, EntityMeta*>& entities) {
        StartPoseObj = {fa.startPose.x, fa.startPose.y, NormalizeReloPushYaw(fa.startPose.yaw)};
        GoalPoseObj = {fa.goalPose.x, fa.goalPose.y, NormalizeReloPushYaw(fa.goalPose.yaw)};
        targetObject = dynamic_cast<ObjectMeta*>(entities.at(fa.object.name));
        sourcePrePushDistance = fa.snapshot.parameters.PrePush_dist;

        // Use the first executable segment start so tasks with obstacle relocation
        // do not inherit an unreachable or already-occupied standby goal.
        TaskStartPoseRobot = DetermineTaskStartPoseRobot(fa);

/////////////// need to verify ///////
        // obs relo path
        firstApproachPath = fa.firstApproachPath;
        obsReloPaths = fa.obsReloPaths;
        obsReloUpdate = fa.obsReloUpdate;
        vertexChain = fa.vertexChain;

        if (obsReloPaths && !obsReloPaths->empty() && vertexChain.size() > 2) {
            auto it = entities.find(vertexChain[1].name);
            if (it != entities.end()) {
                initialApproachEntity = it->second;
            }
        }
        if (!initialApproachEntity) {
            initialApproachEntity = targetObject;
        }

        // Extract obs sequence from vertexChain (objects before target)
        std::vector<std::string> obs_sequence; // todo: where to save this?
        for (const auto& v : vertexChain) {
            if (v.type == VertexType::OBJECT_VERTEX && v.name != fa.object.name) {
                obs_sequence.push_back(v.name);
            }
        }
///////////////////
        EdgePaths.clear();
        // parse trajectories
        //  each edge-path
        for(auto& epath : fa.paths)
        {
            // trajectory (normal: one transfer, prerelo: transfer-transit-transfer)
            for(auto& path : epath.paths)
            {
                auto traj_in = ReloPushPath2TrajPtr(path, nullptr, targetObject,
                                                    0.0, sourcePrePushDistance);
                EdgePaths.emplace_back(traj_in); // time not assigned yet (needs robot first)
            }
        }
    }
/*
    Pose calcRobotPoseFromObj(const Pose& pose_in) {
        if (!assignedRobot || !targetObject) {
            std::cerr << "Assigned robot or target object not set." << std::endl;
            return {0.0, 0.0, 0.0};
        }
        double offset = assignedRobot->size.front_length + targetObject->size.rear_length + 0.1;
        Pose robot_pose;
        robot_pose.x = pose_in.x - offset * std::cos(pose_in.yaw);
        robot_pose.y = pose_in.y - offset * std::sin(pose_in.yaw);
        robot_pose.yaw = pose_in.yaw;
        return robot_pose;
    }

    // todo: need to choose which one to use: Pose from path or this function
    Pose calcStartPoseRobot() {
        if (!assignedRobot || !targetObject) {
            std::cerr << "Assigned robot or target object not set." << std::endl;
            return {0.0, 0.0, 0.0};
        }
        Pose attached = calcRobotPoseFromObj(StartPoseObj);
        double extra_offset = 0.05;
        attached.x -= extra_offset * std::cos(attached.yaw);
        attached.y -= extra_offset * std::sin(attached.yaw);
        return attached;
    }
*/
};

#endif // TASK_H
