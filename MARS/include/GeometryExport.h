#ifndef GEOMETRY_EXPORT_H
#define GEOMETRY_EXPORT_H

#include <PHAstar/Entities.h> // RobotMeta

#include <string>
#include <vector>

// ==========================================
// Instance geometry exporter, for Framework B (see
// MARS/09rl-pretrained-study-plan.md section 3.2). One-time per instance:
// dumps task start/goal poses, reference path polylines resampled to K
// points with per-point push/transit flags, tau_fixed, workspace boundary,
// robot initial poses, and reference order, as JSON.
// ==========================================

struct FinalAllocation; // forward declaration; only extract_ordered_task_path()
                        // and export_instance_geometry() need the full type

struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool is_pushing = false;
};

// Resamples an ordered, temporally-consistent polyline to exactly K points
// via arclength interpolation. x,y are linearly interpolated between the
// bracketing waypoints; yaw and is_pushing are taken from whichever
// bracketing waypoint is arclength-nearer (tie -> the earlier one) -- not
// interpolated, since yaw wraps around and is_pushing is a per-segment (not
// per-point) property.
std::vector<PathPoint> resample_to_k_points(
    const std::vector<PathPoint> &ordered_path, int k);

// Extracts one task's full temporally-ordered path (prerelocations, then
// each edge's sub-paths, in original order) with a push/transit flag per
// point. Unlike DqnFeaturesV2::TaskGeom (which splits push_pts/transit_pts
// into two arrays, discarding temporal order), every waypoint of every
// sub-path is appended in encountered order. Excludes
// firstApproachPath/edgeTransitPaths (robot/sequence-specific, not part of
// the task's own reference path), matching DqnFeaturesV2.cpp's exclusion.
std::vector<PathPoint> extract_ordered_task_path(const FinalAllocation &fa);

// Escapes '"', '\', and control characters for embedding in a JSON string
// literal.
std::string json_escape_string(const std::string &s);

// Writes one JSON file describing the instance's geometry. Returns false
// (and logs to std::cerr) on file-open failure.
bool export_instance_geometry(
    const std::string &out_path,
    const std::string &family, int index,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<RobotMeta> &robot_metas,
    int k);

#endif // GEOMETRY_EXPORT_H
