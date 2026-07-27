#ifndef EXECUTED_SCENARIO_SERIALIZATION_H
#define EXECUTED_SCENARIO_SERIALIZATION_H

// Save-and-replay serialization for a scored MARS execution result
// (ExecutedScenario = summary + params + timetable + owned entities). This
// lets a scored plan be dumped to disk while the run is still alive (see
// evaluate_scenario_batch() in AllocationSearch.cpp) and later reloaded and
// handed straight to show_results() (PHAstar/Visualization.h) with no
// re-execution of the allocation/search.
//
// Format overview (all little/host-endian raw binary, then base64-wrapped by
// the top-level helpers at the bottom of this file):
//   int32   format_version (currently 1)
//   -- summary digest (NOT a full AllocationRunSummary round-trip; the plan
//      and per-task rows are not needed to visualize a result, so they are
//      intentionally omitted here) --
//   string  summary.label
//   bool    summary.all_tasks_succeeded
//   int32   summary.successful_tasks
//   int32   summary.failed_tasks
//   double  summary.makespan
//   -- Params (all scalar fields, in declaration order) --
//   -- entities: count, then per entity: name, EntityType tag, initial_pose,
//      size, then type-specific fields (RobotMeta or ObjectMeta) --
//   -- TimeTable: time_increment, then per_entity_table (entity NAME ->
//      {time -> Pose} map, not pointer-keyed), then trajectory_spans (entity
//      NAME + optional transferred_object NAME, not pointer-keyed) --
//
// Pointer-keyed containers (TimeTable::per_entity_table,
// TimeTable::trajectory_spans, and the entities map itself) are all
// serialized BY ENTITY NAME. On load, entities are rebuilt first (freshly
// `new`'d, owned by the returned ExecutedScenario's destructor via
// cleanup_entities()), and only then are the pointer-keyed containers
// rehydrated by looking up each name in that freshly-built map.

#include <PHAstarPushDemoTypes.h> // ExecutedScenario, Params, TimeTable, Entities
#include <ReloPush/SerializeFinalSequence.h> // Serialization:: primitives (int/double/bool/size_t/string)
#include <ReloPush/base64.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace Serialization
{

// ===========================================================================
// Pose / OccuRect
// ===========================================================================

inline void serialize(std::ostream &os, const Pose &p)
{
  serialize(os, p.x);
  serialize(os, p.y);
  serialize(os, p.yaw);
}

inline void deserialize(std::istream &is, Pose &p)
{
  deserialize(is, p.x);
  deserialize(is, p.y);
  deserialize(is, p.yaw);
}

inline void serialize(std::ostream &os, const OccuRect &r)
{
  serialize(os, r.front_length);
  serialize(os, r.rear_length);
  serialize(os, r.width);
}

inline void deserialize(std::istream &is, OccuRect &r)
{
  deserialize(is, r.front_length);
  deserialize(is, r.rear_length);
  deserialize(is, r.width);
}

// ===========================================================================
// Params (MARS/include/PHAstar/Params.h -- all fields are scalar)
// ===========================================================================

inline void serialize(std::ostream &os, const Params &pm)
{
  serialize(os, pm.xy_resolution);
  serialize(os, pm.yaw_resolution);
  serialize(os, pm.time_step);
  serialize(os, pm.time_resolution);
  serialize(os, pm.min_x);
  serialize(os, pm.min_y);
  serialize(os, pm.max_x);
  serialize(os, pm.max_y);
  serialize(os, pm.max_steer);
  serialize(os, pm.turn_penalty);
  serialize(os, pm.reverse_penalty);
  serialize(os, pm.switch_penalty);
  serialize(os, pm.wait_penalty);
  serialize(os, pm.max_time);
  serialize(os, pm.collision_steps);
  serialize(os, pm.collision_check_time_step);
  serialize(os, pm.analytic_threshold_scale);
  serialize(os, pm.analytic_threshold);
  serialize(os, pm.rs_step_size);
  serialize(os, pm.inflation);
  serialize(os, pm.safety_margin);
  serialize(os, pm.robot_collision_inflation);
  serialize(os, pm.movement_length);
  serialize(os, pm.final_push_distance);
  serialize(os, pm.robot_boundary_origin_only);
  serialize(os, pm.spatial_only_index);
  serialize(os, pm.disable_wait_primitive);
  serialize(os, pm.enable_holonomic_heuristic);
  serialize(os, pm.holonomic_heuristic_resolution);
}

inline void deserialize(std::istream &is, Params &pm)
{
  deserialize(is, pm.xy_resolution);
  deserialize(is, pm.yaw_resolution);
  deserialize(is, pm.time_step);
  deserialize(is, pm.time_resolution);
  deserialize(is, pm.min_x);
  deserialize(is, pm.min_y);
  deserialize(is, pm.max_x);
  deserialize(is, pm.max_y);
  deserialize(is, pm.max_steer);
  deserialize(is, pm.turn_penalty);
  deserialize(is, pm.reverse_penalty);
  deserialize(is, pm.switch_penalty);
  deserialize(is, pm.wait_penalty);
  deserialize(is, pm.max_time);
  deserialize(is, pm.collision_steps);
  deserialize(is, pm.collision_check_time_step);
  deserialize(is, pm.analytic_threshold_scale);
  deserialize(is, pm.analytic_threshold);
  deserialize(is, pm.rs_step_size);
  deserialize(is, pm.inflation);
  deserialize(is, pm.safety_margin);
  deserialize(is, pm.robot_collision_inflation);
  deserialize(is, pm.movement_length);
  deserialize(is, pm.final_push_distance);
  deserialize(is, pm.robot_boundary_origin_only);
  deserialize(is, pm.spatial_only_index);
  deserialize(is, pm.disable_wait_primitive);
  deserialize(is, pm.enable_holonomic_heuristic);
  deserialize(is, pm.holonomic_heuristic_resolution);
}

// ===========================================================================
// EntityMeta / RobotMeta / ObjectMeta
//
// EntityMeta is polymorphic and abstract-ish (RobotMeta/ObjectMeta are the
// only two concrete kinds -- see EntityType), so unlike the plain-old-data
// types above this cannot be a symmetric serialize(T&)/deserialize(T&) pair:
// the deserializer has to pick which concrete subtype to `new` before it can
// fill anything in. Named serialize_entity/deserialize_entity instead.
// ===========================================================================

inline void serialize_entity(std::ostream &os, const EntityMeta &ent)
{
  serialize(os, ent.name);
  serialize(os, static_cast<int>(ent.type));
  serialize(os, ent.initial_pose);
  serialize(os, ent.size);

  if (ent.type == EntityType::ROBOT)
  {
    const auto &r = static_cast<const RobotMeta &>(ent);
    serialize(os, r.min_turning_radius);
    serialize(os, r.min_turning_radius_transit);
    serialize(os, r.min_turning_radius_transfer);
    serialize(os, r.wheel_base);
    serialize(os, r.speed_transit);
    serialize(os, r.speed_transfer);
  }
  else
  {
    const auto &o = static_cast<const ObjectMeta &>(ent);
    serialize(os, o.goal_pose);
  }
}

// Allocates a new RobotMeta or ObjectMeta with `new` (matching the
// initialize_entities()/collect_robot_metas() pattern used everywhere else
// in MARS -- see AllocationSearch.cpp/ReloPushSetup.cpp). Caller owns the
// returned pointer; ExecutedScenario's cleanup_entities() is the intended
// eventual owner.
inline EntityMeta *deserialize_entity(std::istream &is)
{
  std::string name;
  deserialize(is, name);
  int type_i = 0;
  deserialize(is, type_i);
  const EntityType type = static_cast<EntityType>(type_i);

  Pose initial_pose;
  deserialize(is, initial_pose);
  OccuRect size;
  deserialize(is, size);

  if (type == EntityType::ROBOT)
  {
    RobotMeta *r = new RobotMeta();
    r->name = name;
    r->type = EntityType::ROBOT;
    r->initial_pose = initial_pose;
    r->size = size;
    deserialize(is, r->min_turning_radius);
    deserialize(is, r->min_turning_radius_transit);
    deserialize(is, r->min_turning_radius_transfer);
    deserialize(is, r->wheel_base);
    deserialize(is, r->speed_transit);
    deserialize(is, r->speed_transfer);
    return r;
  }

  ObjectMeta *o = new ObjectMeta();
  o->name = name;
  o->type = EntityType::OBJECT;
  o->initial_pose = initial_pose;
  o->size = size;
  deserialize(is, o->goal_pose);
  return o;
}

// entities map: unordered_map<std::string, EntityMeta*>. Not named
// serialize/deserialize (unlike the scalar types above) because the existing
// generic Serialization::serialize/deserialize(unordered_map<string,V>)
// templates from SerializeFinalSequence.h cannot be reused for V=EntityMeta*
// -- they would try (and fail) to serialize a raw, possibly-polymorphic
// pointer field-by-field with no owning/allocation semantics.
inline void serialize_entities(std::ostream &os,
                               const std::unordered_map<std::string, EntityMeta *> &entities)
{
  serialize(os, entities.size());
  for (const auto &[name, ent] : entities)
  {
    (void)name; // entity name is also stored inside serialize_entity()
    serialize_entity(os, *ent);
  }
}

// Populates `entities` with freshly-`new`'d entities keyed by name.
// `entities` is cleared first; on any exception partway through, whatever
// was already inserted remains in `entities` for the caller's
// ExecutedScenario destructor to clean up (no leak).
inline void deserialize_entities(std::istream &is,
                                 std::unordered_map<std::string, EntityMeta *> &entities)
{
  entities.clear();
  std::size_t count = 0;
  deserialize(is, count);
  for (std::size_t i = 0; i < count; ++i)
  {
    EntityMeta *ent = deserialize_entity(is);
    entities[ent->name] = ent;
  }
}

// ===========================================================================
// TimeTable
//
// per_entity_table and trajectory_spans are private and keyed on live
// EntityMeta* pointers (see TimeTable::load_serialized_state() in
// include/PHAstar/TimeTable.h). Serialized BY ENTITY NAME; deserialization
// requires the already-rebuilt `entities` map (name -> EntityMeta*) so it can
// resolve each name back to the pointer that owns it.
// ===========================================================================

inline void serialize_timetable(std::ostream &os, const TimeTable &tt)
{
  serialize(os, tt.time_increment);

  const auto &db = tt.get_database();
  serialize(os, db.size());
  for (const auto &[ent, pose_map] : db)
  {
    serialize(os, ent ? ent->name : std::string());
    serialize(os, pose_map.size());
    for (const auto &[t, pose] : pose_map)
    {
      serialize(os, t);
      serialize(os, pose);
    }
  }

  const auto &spans = tt.get_trajectory_spans();
  serialize(os, spans.size());
  for (const auto &span : spans)
  {
    serialize(os, span.entity ? span.entity->name : std::string());
    const bool has_obj = span.transferred_object != nullptr;
    serialize(os, has_obj);
    if (has_obj)
      serialize(os, span.transferred_object->name);
    serialize(os, span.start_time);
    serialize(os, span.end_time);
    serialize(os, span.is_transfer);
    serialize(os, static_cast<int>(span.kind));
  }
}

// Throws std::runtime_error if a serialized entity name is not present in
// `entities` (a corrupt file, or one saved against a different instance).
inline void deserialize_timetable(std::istream &is, TimeTable &tt,
                                  const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto resolve = [&entities](const std::string &name) -> EntityMeta *
  {
    if (name.empty())
      return nullptr;
    auto it = entities.find(name);
    if (it == entities.end())
      throw std::runtime_error(
          "deserialize_timetable: unknown entity name '" + name + "'");
    return it->second;
  };

  double time_increment = 0.5;
  deserialize(is, time_increment);

  std::unordered_map<EntityMeta *, std::map<double, Pose>> per_entity_table;
  std::size_t entity_count = 0;
  deserialize(is, entity_count);
  for (std::size_t i = 0; i < entity_count; ++i)
  {
    std::string name;
    deserialize(is, name);
    std::size_t pose_count = 0;
    deserialize(is, pose_count);
    std::map<double, Pose> pose_map;
    for (std::size_t j = 0; j < pose_count; ++j)
    {
      double t = 0.0;
      Pose pose;
      deserialize(is, t);
      deserialize(is, pose);
      pose_map[t] = pose;
    }
    per_entity_table[resolve(name)] = std::move(pose_map);
  }

  std::vector<TimeTable::TrajectorySpan> spans;
  std::size_t span_count = 0;
  deserialize(is, span_count);
  spans.reserve(span_count);
  for (std::size_t i = 0; i < span_count; ++i)
  {
    std::string ent_name;
    deserialize(is, ent_name);
    bool has_obj = false;
    deserialize(is, has_obj);
    std::string obj_name;
    if (has_obj)
      deserialize(is, obj_name);

    TimeTable::TrajectorySpan span;
    span.entity = resolve(ent_name);
    span.transferred_object = has_obj ? resolve(obj_name) : nullptr;
    deserialize(is, span.start_time);
    deserialize(is, span.end_time);
    deserialize(is, span.is_transfer);
    int kind_i = 0;
    deserialize(is, kind_i);
    span.kind = static_cast<TrajectoryKind>(kind_i);
    spans.push_back(span);
  }

  tt.load_serialized_state(time_increment, std::move(per_entity_table), std::move(spans));
}

} // namespace Serialization

// ===========================================================================
// Top-level ExecutedScenario helpers
// ===========================================================================

namespace ExecutedScenarioSerialization
{
inline constexpr int kFormatVersion = 1;
}

// Serializes `scn` to a base64-wrapped binary blob. Safe to call while `scn`
// is still alive/in-scope (e.g. inside evaluate_scenario_batch(), before the
// ExecutedScenario leaves scope and its entities are deleted).
inline std::string serialize_executed_scenario_b64(const ExecutedScenario &scn)
{
  std::ostringstream oss(std::ios::binary);
  Serialization::serialize(oss, ExecutedScenarioSerialization::kFormatVersion);

  // Summary digest -- intentionally NOT a full AllocationRunSummary
  // round-trip (see file header comment): just enough to report the score
  // alongside the replay.
  Serialization::serialize(oss, scn.summary.label);
  Serialization::serialize(oss, scn.summary.all_tasks_succeeded);
  Serialization::serialize(oss, scn.summary.successful_tasks);
  Serialization::serialize(oss, scn.summary.failed_tasks);
  Serialization::serialize(oss, scn.summary.makespan);

  Serialization::serialize(oss, scn.params);
  Serialization::serialize_entities(oss, scn.entities);
  Serialization::serialize_timetable(oss, scn.timetable);

  return base64_encode(oss.str());
}

// Inverse of serialize_executed_scenario_b64(). The returned ExecutedScenario
// owns freshly-`new`'d entities (its destructor -- cleanup_entities() --
// deletes them; no double-free since these are brand-new pointers, never
// shared with anything else). Throws std::runtime_error on a corrupt blob or
// an unrecognized format_version.
inline ExecutedScenario deserialize_executed_scenario_b64(const std::string &b64)
{
  const std::string binary = base64_decode(b64);
  std::istringstream iss(binary, std::ios::binary);

  int format_version = 0;
  Serialization::deserialize(iss, format_version);
  if (format_version != ExecutedScenarioSerialization::kFormatVersion)
  {
    throw std::runtime_error(
        "deserialize_executed_scenario_b64: unsupported format_version " +
        std::to_string(format_version));
  }

  ExecutedScenario scn;
  Serialization::deserialize(iss, scn.summary.label);
  Serialization::deserialize(iss, scn.summary.all_tasks_succeeded);
  Serialization::deserialize(iss, scn.summary.successful_tasks);
  Serialization::deserialize(iss, scn.summary.failed_tasks);
  Serialization::deserialize(iss, scn.summary.makespan);

  Serialization::deserialize(iss, scn.params);
  Serialization::deserialize_entities(iss, scn.entities);
  Serialization::deserialize_timetable(iss, scn.timetable, scn.entities);

  return scn;
}

#endif // EXECUTED_SCENARIO_SERIALIZATION_H
