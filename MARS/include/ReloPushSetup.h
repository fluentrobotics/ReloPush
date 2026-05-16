#ifndef RELOPUSH_SETUP_H
#define RELOPUSH_SETUP_H

#include <PHAstar/Point.h>     // for Pose
#include <PHAstar/Entities.h>  // for RobotMeta, EntityMeta
#include <PHAstar/TimeTable.h> // for TimeTable, Trajectory
#include <ReloPush/FinalSequenceHandoff.h>  // for FinalAllocation, HandoffInstanceInfo
#include <cstdint>
#include <string>
#include <vector>

// Forward declarations
struct RuntimeOptions;
struct Params;
struct EdgePath;

// ==========================================
// ReloPush Single-Robot Setup Helpers
// ==========================================

double compute_relopush_single_robot_makespan(
    const std::vector<FinalAllocation> &loaded_sequence);

RobotMeta *make_relopush_robot();

bool find_first_relopush_robot_pose(
    const std::vector<FinalAllocation> &loaded_sequence,
    Pose &out_pose);

Trajectory make_relopush_trajectory(const ReloPush::StatePathPtr &path,
                                     bool is_transfer,
                                     RobotMeta *robot,
                                     EntityMeta *transferred_object,
                                     double start_time);

void append_relopush_trajectory(TimeTable &timetable,
                                 RobotMeta *robot,
                                 EntityMeta *transferred_object,
                                 const ReloPush::StatePathPtr &path,
                                 bool is_transfer,
                                 double &current_time);

void append_relopush_edge_path(TimeTable &timetable,
                                RobotMeta *robot,
                                EntityMeta *transferred_object,
                                const EdgePath &edge_path,
                                double &current_time);

void visualize_relopush_plan(
    int argc, char **argv,
    const std::vector<FinalAllocation> &loaded_sequence,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const RuntimeOptions &options);

ReloPush::HandoffInstanceInfo default_instance_info(const RuntimeOptions &options);

std::uint32_t mix_seed(std::uint32_t seed, std::uint32_t salt);

#endif // RELOPUSH_SETUP_H
