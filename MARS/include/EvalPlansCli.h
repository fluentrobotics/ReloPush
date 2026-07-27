#ifndef EVAL_PLANS_CLI_H
#define EVAL_PLANS_CLI_H

#include <PHAstarPushDemoOptions.h> // RuntimeOptions

#include <cstddef>
#include <string>
#include <vector>

// ==========================================
// Batch-oracle CLI for the policy-gradient PoC (see
// MARS/14pg-poc-design.md section 4, "Batch oracle CLI", and the P0 build
// item). Accepts complete (task order, robot assignment) plans and returns
// per-plan feasibility/makespan/first-fail-step/executed-robot through the
// exact same evaluate_scenario_batch() pathway the DQN/LNS search uses
// (early-abort per options.early_abort_eval_on_failure). All parsing/
// formatting helpers below are pure functions so they are directly
// unit-testable without constructing a real FinalAllocation instance; see
// MARS/tests/phastar_unit_tests.cpp.
// ==========================================

struct FinalAllocation; // forward declaration; only run_eval_plans_cli() needs
                        // the full type

// One parsed input line: {"id": <string>, "order": [task indices in global
// priority order], "assign": [robot index per task, aligned with "order"]}.
// Robot index i maps to name "robot{i+1}" -- see assignment_to_plan_preferences.
struct EvalPlanRequest
{
  std::string id;
  std::vector<std::size_t> order;
  std::vector<std::size_t> assign; // aligned with `order` (assign[k] is the
                                   // robot for order[k]), NOT indexed by
                                   // original task index.
};

// Parses one JSONL line into `out`. This is a small hand-rolled parser for
// the fixed {"id","order","assign"} schema above (this codebase uses no JSON
// library anywhere -- see GeometryExport.cpp's write-side equivalent), not a
// general JSON parser: it locates each key via a top-level substring search,
// so it would mis-parse a value containing a literal `"order"`-shaped
// substring, which the fixed schema here never produces. Returns false (and
// sets `error`) if any of the three keys is missing, the "id" value is not a
// well-formed JSON string, "order"/"assign" are not well-formed flat arrays
// of non-negative integers, or "order"/"assign" differ in length.
bool parse_eval_plan_line(const std::string &line, EvalPlanRequest &out,
                          std::string &error);

// True iff `order` is a complete permutation of [0, task_count): validates
// length, range, and absence of duplicates (the same three checks
// PHAstar_push_demo.cpp's --fixed-order mode already applies). Sets `error`
// on the first violation found.
bool validate_plan_order(const std::vector<std::size_t> &order,
                         std::size_t task_count, std::string &error);

// Inverse of the "robot{i+1}" naming convention: "robot3" -> 2, "" -> -1,
// "robot0" -> -1 (1-based names only), anything not matching "robot<digits>"
// -> -1.
long parse_robot_index_from_name(const std::string &name);

// RFC4180-style CSV field escaping: wraps in double quotes (doubling any
// embedded quote) iff the field contains a comma, quote, or newline;
// returned unchanged otherwise.
std::string csv_escape_field(const std::string &s);

// Reads `input_path` (one JSON object per line, see EvalPlanRequest), skips
// blank lines, and evaluates every well-formed complete plan in ONE
// evaluate_scenario_batch() call (so options.lns_threads-way parallelism
// applies exactly as it does for the DQN/LNS search), then writes
// `output_path` as CSV:
//   id, feasible (0/1, or -1 for a malformed/invalid line), makespan (or -1
//   if infeasible or malformed), first_fail_step (0-based position in
//   `order`, or -1 if feasible/malformed), executed_robots (semicolon-joined
//   robot indices aligned with `order`; -1 where a task never executed --
//   i.e. any TaskCsvRow whose status is not "SUCCESS", including every task
//   past an early-abort cutoff; empty for a malformed line),
//   eval_wall_s (the batch's total wall-clock time divided evenly across
//   every successfully-submitted plan in this call -- evaluate_scenario_batch
//   evaluates its whole input concurrently across options.lns_threads
//   workers, so a true per-plan wall time is not separately observable; 0
//   for a malformed line).
// A malformed/invalid line (bad JSON, wrong "order"/"assign" length,
// out-of-range or duplicate task index) is skipped (not submitted for
// evaluation), logged to stderr with the 1-based line number, and still
// produces one feasible=-1 CSV row -- never aborts the batch. Returns false
// (logging to stderr) only on an I/O failure (unreadable input_path,
// unwritable output_path).
//
// Save-and-replay (see MARS/16save-replay-implementation.md): if
// options.eval_plans_result_out_dir is non-empty, every evaluated plan's
// ExecutedScenario (already serialized by evaluate_scenario_batch into
// ScenarioEvaluationResult::serialized_result) is written to
// "<dir>/<id>.scn.b64". If options.eval_plans_result_out_path is non-empty,
// only the single best FEASIBLE plan (min makespan, ties broken by first
// occurrence in input order) is written to that one file. Either, both, or
// neither may be set; neither is the default (no behavior change).
//
// Stage 1 planner timing instrumentation (opt-in; see PlanTimingStats in
// PHAstarPushDemoTypes.h): if options.eval_plans_timing_out_path is
// non-empty, an additional CSV is written to that path with one row per
// evaluated plan (same "id" as the main output CSV; a malformed/invalid
// line gets an all-zero row, same as its main-CSV counterpart) and a header
// of "id" followed by every PlanTimingStats field in its declared order
// (true_wall_s, search_wall_s_primary/fine/contact/other,
// n_searches_primary/fine/contact/other, search_iterations_total,
// n_search_cap_hits, heuristic_time_s, primitive_collision_time_s,
// analytic_validation_time_s, holonomic_heuristic_time_s, sched_wall_s,
// n_find_safe_start_calls, n_start_candidates_tried, traj_scan_wall_s,
// terminal_hold_wall_s, safe_parking_wall_s, n_parking_relocations,
// n_robot_candidate_attempts, n_post_validation_retries,
// n_obsrelo_segments). Empty (default) = no extra file written; the
// PlanTimingStats accumulation itself always happens inside
// evaluate_scenario_batch() regardless of this flag (cheap: chrono +
// counters only), so this flag only gates the extra CSV write.
bool run_eval_plans_cli(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const std::string &input_path,
    const std::string &output_path);

#endif // EVAL_PLANS_CLI_H
