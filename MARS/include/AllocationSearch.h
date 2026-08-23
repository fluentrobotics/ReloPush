#ifndef ALLOCATION_SEARCH_H
#define ALLOCATION_SEARCH_H

#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <ReloPush/TaskAllocation.hpp>
#include <PHAstar/TimeTable.h>
#include <PHAstar/Entities.h>
#include <Task.h>
#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations
class Params;

// ==========================================
// Config Display
// ==========================================

void print_runtime_options(const RuntimeOptions &options);

// See definition in AllocationSearch.cpp for the full contract. Exposed here
// (rather than kept file-local) so unit tests can exercise the derivation
// directly.
int effective_worker_expansion_threads(const RuntimeOptions &options);

// ==========================================
// Environment & Task Initialization
// ==========================================

Params initialize_params(const std::vector<FinalAllocation> &loadedSequence,
                         const RuntimeOptions &options);

std::unordered_map<std::string, EntityMeta *>
initialize_entities(const std::vector<FinalAllocation> &loadedSequence,
                    int requested_robot_count,
                    const std::optional<std::vector<std::array<double, 3>>> &robot_poses_override = std::nullopt,
                    const std::optional<std::array<double, 3>> &robot4_pose_override = std::nullopt);

std::vector<Task> initialize_tasks(
    const std::vector<FinalAllocation> &loaded_sequence,
    std::unordered_map<std::string, EntityMeta *> &entities);

AllocationScenarioPlan make_identity_plan(std::size_t task_count);

void initialize_environment(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    std::uint32_t parking_seed,
    Params &params,
    std::unordered_map<std::string, EntityMeta *> &entities,
    TimeTable &timetable,
    std::vector<RobotMeta *> &all_robots,
    bool verbose = true);

std::unordered_map<std::string, RobotMeta *>
build_robot_lookup(const std::vector<RobotMeta *> &all_robots);

std::vector<std::size_t> normalized_task_order(
    const AllocationScenarioPlan &plan,
    std::size_t task_count);

std::vector<Task> build_tasks_for_plan(
    const std::vector<FinalAllocation> &loaded_sequence,
    std::unordered_map<std::string, EntityMeta *> &entities,
    const std::vector<RobotMeta *> &all_robots,
    const AllocationScenarioPlan &plan);

// ==========================================
// Scenario Execution & Summary
// ==========================================

AllocationRunSummary summarize_run(
    const std::string &label,
    const AllocationScenarioPlan &plan,
    std::uint32_t parking_seed,
    const std::vector<TaskCsvRow> &task_rows,
    const TimeTable &timetable);

ExecutedScenario execute_allocation_scenario(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &plan,
    const std::string &label,
    std::uint32_t parking_seed,
    bool verbose = true,
    bool abort_on_first_failure = false,
    PlanTimingStats *plan_stats = nullptr);

std::vector<ScenarioEvaluationResult> evaluate_scenario_batch(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const std::vector<ScenarioEvaluationRequest> &requests);

std::vector<LnsEvaluationResult> evaluate_lns_batch(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<LnsSearchCandidate> &candidates,
    const std::vector<std::string> &robot_names,
    const std::string &label);

ExecutedScenario repair_destroyed_tasks_with_sampled_insertion(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<std::size_t> &destroyed_tasks,
    const std::vector<std::string> &robot_names,
    const std::vector<double> &destroy_scores,
    std::uint32_t parking_seed,
    std::uint32_t repair_seed,
    const std::string &label,
    bool disable_local_silencer = false);

// ==========================================
// Plan Mutation & Comparison
// ==========================================

AllocationScenarioPlan make_assignment_plan_from_greedy(
    const AllocationRunSummary &greedy_summary);

AllocationScenarioPlan mutate_assignment_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::string> &robot_names,
    std::mt19937 &rng);

AllocationScenarioPlan mutate_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng);

AllocationScenarioPlan mutate_full_shuffle_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng);

AllocationScenarioPlan make_distinct_assignment_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::string> &robot_names,
    std::mt19937 &rng);

AllocationScenarioPlan make_distinct_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng);

bool sample_unique_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    AllocationScenarioPlan (*mutator)(const AllocationScenarioPlan &, std::mt19937 &),
    std::mt19937 &rng,
    std::unordered_set<std::string> &tried_signatures,
    const LearnedOrderConstraints *constraints,
    AllocationScenarioPlan *out_plan,
    int max_attempts = 256);

// ==========================================
// Run Comparison Predicates
// ==========================================

bool is_better_run(const AllocationRunSummary &candidate,
                   const AllocationRunSummary &current_best);

bool is_preferred_search_result(const AllocationRunSummary &candidate,
                                const AllocationRunSummary &current_best);

bool same_execution_signature(const AllocationRunSummary &a,
                              const AllocationRunSummary &b);

bool plans_have_same_order(const AllocationScenarioPlan &a,
                           const AllocationScenarioPlan &b);

// ==========================================
// Plan Inspection Utilities
// ==========================================

std::string task_order_signature(const AllocationScenarioPlan &plan);

int count_assignment_preference_changes(const AllocationScenarioPlan &a,
                                        const AllocationScenarioPlan &b);

int count_task_position_changes(const AllocationScenarioPlan &a,
                                const AllocationScenarioPlan &b);

std::string preview_task_order(const AllocationScenarioPlan &plan,
                               const std::vector<FinalAllocation> &loaded_sequence,
                               std::size_t max_items = 6);

std::string preview_assignment_changes(const AllocationScenarioPlan &base_plan,
                                       const AllocationScenarioPlan &candidate_plan,
                                       const std::vector<FinalAllocation> &loaded_sequence,
                                       std::size_t max_items = 4);

// ==========================================
// Order Constraint Learning
// ==========================================

LearnedOrderConstraints make_learned_order_constraints(std::size_t task_count);

void learn_order_constraints_from_failed_summary(
    const AllocationRunSummary &failed_summary,
    const AllocationScenarioPlan &reference_plan,
    const std::vector<FinalAllocation> &loaded_sequence,
    LearnedOrderConstraints &constraints);

// ==========================================
// LNS Destroy Operators
// ==========================================

std::vector<double> compute_task_destroy_scores(
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence);

std::vector<std::size_t> destroy_random_tasks(
    std::size_t task_count,
    std::size_t destroy_count,
    std::mt19937 &rng);

std::vector<std::size_t> destroy_wait_neighborhood_tasks(
    const AllocationScenarioPlan &plan,
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence,
    std::size_t destroy_count,
    std::mt19937 &rng);

std::vector<std::size_t> destroy_critical_suffix_tasks(
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence,
    std::size_t destroy_count,
    std::mt19937 &rng);

// ==========================================
// Print Utilities
// ==========================================

void print_comparison_line(const AllocationRunSummary &summary,
                           double greedy_makespan);

std::vector<std::string> collect_robot_names(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options);

// Same throwaway-environment pattern as collect_robot_names(), but harvests
// full RobotMeta values (speeds, turning radii, size, initial pose) instead
// of just names. Used by the DQN v2 feature path.
std::vector<RobotMeta> collect_robot_metas(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options);

AllocationRunSummary make_placeholder_summary(
    const std::string &label,
    const AllocationScenarioPlan &plan);

std::string format_feasible_makespan_list(
    const std::vector<SearchTrialRecord> &records);

void print_search_method_summary(
    const std::string &label,
    bool has_feasible_candidate,
    const AllocationRunSummary &best_feasible,
    bool has_partial_candidate,
    const AllocationRunSummary &best_partial,
    double greedy_makespan);

#endif // ALLOCATION_SEARCH_H
