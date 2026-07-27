#ifndef SAFE_PARKING_H
#define SAFE_PARKING_H

#include <PHAstar/Point.h>          // for Pose
#include <PHAstar/Entities.h>       // for EntityMeta, RobotMeta
#include <PHAstar/Params.h>         // for Params
#include <PHAstar/Utils.h>          // for kDubinsTwoPi, mod2pi
#include <PHAstarPushDemoTypes.h>    // for ParkingCandidate
#include <PHAstarPushDemoOptions.h>  // for ParkingCandidateMode
#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations
class TimeTable;
struct Trajectory;
struct SafeParkingDebugTrial;
struct RuntimeOptions;

// ==========================================
// Parking RNG & Global State
// ==========================================

extern thread_local ParkingCandidateMode g_parking_candidate_mode;

const char *parking_candidate_mode_name(ParkingCandidateMode mode);
const char *parking_candidate_mode_name();  // Uses g_parking_candidate_mode

void initialize_parking_rng(bool has_fixed_seed, std::uint32_t fixed_seed);
std::mt19937 &parking_rng();
std::uint32_t parking_rng_seed();

// ==========================================
// Safe Parking Candidate Generation
// ==========================================

std::vector<ParkingCandidate>
generate_parking_candidates_randomized(const Pose &current_pose,
                                       RobotMeta *robot,
                                       const Params &params);

Pose propagate_pose_with_primitive(const Pose &pose,
                                    int direction,
                                    double steer,
                                    double speed,
                                    double wheel_base,
                                    double dt);

std::vector<ParkingCandidate>
generate_parking_candidates_expand_primitives(const Pose &current_pose,
                                               RobotMeta *robot,
                                               const Params &params);

std::vector<ParkingCandidate>
generate_parking_candidates_connected_primitives(
    const Pose &current_pose, RobotMeta *robot,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    double start_time,
    PlanTimingStats *plan_stats = nullptr);

std::vector<ParkingCandidate>
generate_parking_candidates_reverse_recent_path(
    const Pose &current_pose, RobotMeta *robot,
    TimeTable &timetable,
    double start_time);

std::vector<ParkingCandidate>
generate_parking_candidates(const Pose &current_pose, RobotMeta *robot,
                            const Params &params,
                            TimeTable *timetable = nullptr,
                            const std::unordered_map<std::string, EntityMeta *> *entities = nullptr,
                            double start_time = 0.0,
                            PlanTimingStats *plan_stats = nullptr);

std::vector<ParkingCandidate>
generate_parking_candidates_for_mode(const Pose &current_pose,
                                     RobotMeta *robot,
                                     const Params &params,
                                     ParkingCandidateMode mode,
                                     TimeTable *timetable = nullptr,
                                     const std::unordered_map<std::string, EntityMeta *> *entities = nullptr,
                                     double start_time = 0.0,
                                     PlanTimingStats *plan_stats = nullptr);

bool is_pose_collision_free_at_time(EntityMeta *entity, const Pose &pose,
                                    double t, TimeTable &timetable,
                                    const Params &params);

bool is_parking_pose_safe_until_last_timestamp(EntityMeta *entity,
                                                const Pose &pose,
                                                double from_t,
                                                TimeTable &timetable,
                                                const Params &params,
                                                double step = 0.5);

// ==========================================
// Parking Geometry Helpers
// ==========================================

double parking_entity_effective_radius(EntityMeta *ent, const Params &params);
double ray_distance_to_boundary(const Pose &origin, double angle, const Params &params);

struct ParkingDirectionHistogram
{
  std::vector<double> density;
  double max_density = 0.0;

  bool empty() const
  {
    return density.empty() || max_density <= 1e-9;
  }

  double sample(double angle) const
  {
    if (density.empty())
      return 0.0;

    double wrapped = mod2pi(angle);
    double scaled = wrapped / kDubinsTwoPi * static_cast<double>(density.size());
    int idx0 = static_cast<int>(std::floor(scaled)) % static_cast<int>(density.size());
    if (idx0 < 0)
      idx0 += static_cast<int>(density.size());
    int idx1 = (idx0 + 1) % static_cast<int>(density.size());
    double frac = scaled - std::floor(scaled);
    return density[idx0] * (1.0 - frac) + density[idx1] * frac;
  }

  double sample_normalized(double angle) const
  {
    if (empty())
      return 0.0;
    return sample(angle) / max_density;
  }
};

ParkingDirectionHistogram build_parking_direction_histogram(
    RobotMeta *blocker,
    const Pose &start_pose,
    double ready_time,
    TimeTable &timetable,
    const Params &params);

struct ConnectedSafeParkingSearchResult
{
  bool found = false;
  Pose parking_pose;
  TimeTable committed_timetable;
};

// `hint_reference_time` is forwarded to parking_candidate_clears_blocked_hint
// as the real scheduling-window time to validate blocked_traj_hint against
// (see CollisionScheduling.h for details); pass -1.0 if unavailable.
ConnectedSafeParkingSearchResult search_safe_parking_connected_search(
    ParkingCandidateMode mode,
    RobotMeta *blocker,
    const Pose &start_pose,
    double ready_time,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Trajectory *blocked_traj_hint,
    double hint_reference_time,
    std::vector<SafeParkingDebugTrial> *debug_trials,
    PlanTimingStats *plan_stats = nullptr);

// Thread-local relocation cache (defined in PHAstar_push_demo.cpp, used by relocate_blocking_robot)
extern thread_local std::unordered_map<std::string, double> g_recent_failed_relocations;
std::unordered_map<std::string, double> &recent_failed_relocation_cache();

// `context` tags the [Relocate] diagnostic log lines to distinguish
// mechanism A (relocating an actual idle blocker) from mechanism B
// (self-parking the robot currently being planned) — purely diagnostic,
// does not affect relocation behavior.
// `hint_reference_time` is the real scheduling-window time blocked_traj_hint
// must be validated against (see parking_candidate_clears_blocked_hint in
// CollisionScheduling.h); pass -1.0 (default) when no better time is known,
// which preserves the old best-effort fallback behavior.
// `plan_stats`, when non-null, times the whole call into
// PlanTimingStats::safe_parking_wall_s (every invocation, regardless of
// outcome) and increments PlanTimingStats::n_parking_relocations on success;
// it is also forwarded to the internal candidate-search machinery (see
// search_safe_parking_connected_search / generate_parking_candidates_for_mode
// and the PHAStar relocation search inside this function), which additionally
// tag their own search wall time into PlanTimingStats::search_wall_s_other.
bool relocate_blocking_robot(RobotMeta *blocker, TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    const Trajectory *blocked_traj_hint = nullptr,
    double hint_reference_time = -1.0,
    const char *context = "blocker",
    PlanTimingStats *plan_stats = nullptr);

#endif // SAFE_PARKING_H
