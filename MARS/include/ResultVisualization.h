#ifndef RESULT_VISUALIZATION_H
#define RESULT_VISUALIZATION_H

#include <PHAstar/Point.h>     // for Pose
#include <PHAstar/Entities.h>  // for EntityMeta, EntityType
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations
struct Params;
class TimeTable;
struct RuntimeOptions;
namespace ReloPush {
  struct HandoffInstanceInfo;
}

class QPainter;
class QColor;
class QPen;

// ==========================================
// Result Summary Figure (QPainter)
// ==========================================

// Helper: sort entities by type for consistent rendering
std::vector<EntityMeta *> sorted_entities_by_type(
    const std::unordered_map<std::string, EntityMeta *> &entities,
    EntityType type);

// Main entry point
void export_result_summary_figure(
    const RuntimeOptions &options,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params);

#endif // RESULT_VISUALIZATION_H
