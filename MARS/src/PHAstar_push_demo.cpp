/*****************************************************************
 * Prioritized Hybrid Astar Demo with Pushing Tasks from ReloPush
 * Main Entry Point — Refactored for Debuggability & Modularity
 *
 * 2025.11.9
 * Orchestrator Version
 ******************************************************************/

#include <PHAstar/PHAstar.h>
#include <PHAstar/Reeds_Shepp.h>
#include <Task.h>
#include <PHAstar/Visualization.h>
#include <PHAstar/CollisionUtils.h>
#include <PHAstarPushDemoOptions.h>
#include <PHAstarPushDemoTypes.h>
#include <ReloPush/FinalSequenceHandoff.h>
#include <ReloPush/config.h>
#include <ReloPush/TaskAllocation.hpp>
#include <ReloPush/PrintInColor.hpp>
#include <CsvLogging.h>
#include <ResultVisualization.h>
#include <SafeParking.h>
#include <PlanningHelpers.h>
#include <CollisionScheduling.h>
#include <ReloPushSetup.h>
#include <AllocationSearch.h>
#include <TaskExecution.h>
#include <RuntimeOptionsParsing.h>
#include <DataLoading.h>
#include <SearchOrchestrator.h>
#include <DqnAllocationSearch.h>
#include <TransitionLogger.h>
#include <GeometryExport.h>
#include <PgTableExport.h>
#include <EvalPlansCli.h>
#include <ExecutedScenarioSerialization.h>
#include <QFont>
#include <QImage>
#include <QPainterPath>
#include <QPen>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

// for finding parking
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>

bool DEBUG_VIS = false;

// ==========================================
// MAIN ORCHESTRATOR
// ==========================================

int phastar_push_demo_main(int argc, char **argv)
{
    // Parse options and load data
    RuntimeOptions runtime_options = parse_runtime_options(argc, argv);

    // Save-and-replay playback (see MARS/16save-replay-implementation.md):
    // visualize a previously-saved ExecutedScenario with no allocation
    // search at all. Deliberately placed BEFORE load_data() below: this mode
    // has nothing to do with any ReloPush sequence file (the scenario being
    // replayed was already fully executed when it was saved), so it must not
    // fail just because no --sequence-file= was given / the default sequence
    // file is missing.
    if (!runtime_options.play_result_path.empty())
    {
        std::ifstream scn_in(runtime_options.play_result_path, std::ios::binary);
        if (!scn_in.is_open())
        {
            std::cerr << "[PlayResult] Failed to open " << runtime_options.play_result_path
                      << std::endl;
            return 2;
        }
        std::string b64_data((std::istreambuf_iterator<char>(scn_in)),
                             std::istreambuf_iterator<char>());

        ExecutedScenario scn = deserialize_executed_scenario_b64(b64_data);
        std::cout << "[PlayResult] Loaded '" << scn.summary.label << "' (makespan="
                  << scn.summary.makespan << ", feasible="
                  << (scn.summary.all_tasks_succeeded ? "yes" : "no") << ") from "
                  << runtime_options.play_result_path << std::endl;

        show_results(argc, argv, scn.timetable, scn.entities, scn.params);
        return scn.summary.all_tasks_succeeded ? 0 : 1;
    }

    ReloPush::HandoffInstanceInfo instance_info;
    std::vector<FinalAllocation> loadedSequence;
    std::unique_ptr<ReloPush::FinalSequenceHandoffServer> handoff_server;
    if (!load_data(runtime_options, instance_info, loadedSequence, handoff_server))
    {
        return 1;
    }

    const double relopush_single_robot_makespan =
        compute_relopush_single_robot_makespan(loadedSequence);

    // Fixed-order evaluation mode: execute one explicitly-given task order
    // through the real multi-robot executor with no search loop, and report
    // its true makespan/feasibility. Used by external pipelines (e.g. a
    // trained model) that construct a task order and need ground-truth
    // evaluation rather than a proxy. Placed here (right after load_data and
    // the (side-effecting -- see below) single-robot makespan computation,
    // before the greedy baseline / robot_metas collection) since this mode
    // needs neither of the latter.
    //
    // IMPORTANT: compute_relopush_single_robot_makespan() above must run
    // before this block, even though this mode does not use its return
    // value. It has a load-bearing side effect: EdgePath::toStatePath()
    // (include/ReloPush/GraphData.hpp:27) returns a shared_ptr aliasing the
    // *same* underlying path vector on every call, and
    // compute_relopush_single_robot_makespan() (MARS/src/ReloPushSetup.cpp)
    // does an in-place push_back onto that shared vector the first time each
    // path is converted. The real executor (build_tasks_for_plan /
    // execute_task_allocation_loop) reads the same shared path objects, so
    // skipping this call here previously made --fixed-order evaluate a
    // subtly different (unextended) path than every other mode, giving a
    // makespan a few hundredths of a second off. Confirmed empirically:
    // calling this line first is necessary and sufficient to make a fixed
    // identity order reproduce the greedy path's makespan bit-for-bit.
    if (!runtime_options.fixed_order_path_or_list.empty())
    {
        const std::size_t task_count = loadedSequence.size();
        std::vector<std::size_t> fixed_order;
        bool parse_ok = true;
        std::string parse_error;

        std::stringstream ss(runtime_options.fixed_order_path_or_list);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            token.erase(std::remove_if(token.begin(), token.end(),
                                       [](unsigned char ch)
                                       { return std::isspace(ch); }),
                        token.end());
            if (token.empty())
            {
                continue;
            }
            try
            {
                std::size_t parsed_chars = 0;
                unsigned long long value = std::stoull(token, &parsed_chars);
                if (parsed_chars != token.size())
                {
                    throw std::invalid_argument("trailing characters");
                }
                fixed_order.push_back(static_cast<std::size_t>(value));
            }
            catch (const std::exception &)
            {
                parse_ok = false;
                parse_error = "could not parse '" + token + "' as an integer index";
                break;
            }
        }

        if (parse_ok && fixed_order.size() != task_count)
        {
            parse_ok = false;
            std::ostringstream oss;
            oss << "wrong length: got " << fixed_order.size()
                << " indices, expected " << task_count;
            parse_error = oss.str();
        }

        if (parse_ok)
        {
            std::vector<bool> seen(task_count, false);
            for (std::size_t idx : fixed_order)
            {
                if (idx >= task_count)
                {
                    parse_ok = false;
                    std::ostringstream oss;
                    oss << "out-of-range index " << idx << " (valid range is 0.."
                        << (task_count - 1) << ")";
                    parse_error = oss.str();
                    break;
                }
                if (seen[idx])
                {
                    parse_ok = false;
                    std::ostringstream oss;
                    oss << "duplicate index " << idx;
                    parse_error = oss.str();
                    break;
                }
                seen[idx] = true;
            }
        }

        if (!parse_ok)
        {
            std::cerr << "[FixedOrder] Invalid --fixed-order value '"
                      << runtime_options.fixed_order_path_or_list << "': "
                      << parse_error << std::endl;
            return 2;
        }

        AllocationScenarioPlan plan;
        plan.task_order = fixed_order;

        ExecutedScenario result = execute_allocation_scenario(
            loadedSequence, runtime_options, plan, "fixed-order",
            /*parking_seed=*/1u, /*verbose=*/true);

        print_comparison_line(result.summary, result.summary.makespan);

        return result.summary.all_tasks_succeeded ? 0 : 1;
    }

    // Batch-oracle CLI (policy-gradient PoC, MARS/14pg-poc-design.md section
    // 4): evaluate a whole file of externally-constructed (order, assignment)
    // plans through the same evaluate_scenario_batch() pathway the DQN/LNS
    // search uses, with no search loop of its own. Placed here (same scope as
    // --fixed-order above, for the same reason: it needs neither the greedy
    // baseline nor robot_metas) -- see EvalPlansCli.h.
    if (!runtime_options.eval_plans_path.empty())
    {
        if (runtime_options.eval_plans_out_path.empty())
        {
            std::cerr << "[EvalPlans] --eval-plans-out=<path.csv> is required together "
                         "with --eval-plans="
                      << std::endl;
            return 2;
        }

        const bool eval_plans_ok = run_eval_plans_cli(
            loadedSequence, runtime_options, runtime_options.eval_plans_path,
            runtime_options.eval_plans_out_path);
        if (eval_plans_ok)
            std::cout << "[EvalPlans] Wrote results to "
                      << runtime_options.eval_plans_out_path << std::endl;
        else
            std::cerr << "[EvalPlans] Failed to evaluate plans from "
                      << runtime_options.eval_plans_path << std::endl;
        return eval_plans_ok ? 0 : 1;
    }

    DEBUG_VIS = runtime_options.debug_vis;
    // print_runtime_options(runtime_options);
    std::cout << "[Config] Instance file: " << instance_info.file_name << std::endl;
    std::cout << "[Config] Instance index: " << instance_info.instance_index << std::endl;
    std::cout << "[Config] ReloPush single-robot makespan: "
              << std::fixed << std::setprecision(2)
              << relopush_single_robot_makespan << "s" << std::endl;

    const std::string relopush_sequence_path =
        runtime_options.input_sequence_path.empty()
            ? default_sequence_path()
            : runtime_options.input_sequence_path;
    write_relopush_makespan_diagnostics(
        loadedSequence,
        instance_info,
        runtime_options,
        relopush_sequence_path,
        relopush_single_robot_makespan);

    if (runtime_options.visualize_relopush_plan)
    {
        visualize_relopush_plan(argc, argv, loadedSequence, instance_info, runtime_options);
    }

    // Check if greedy-only mode (no search)
    const int improvement_iters =
        (runtime_options.search_improvement_mode == SearchImprovementMode::DQN)
            ? runtime_options.dqn_iterations
            : runtime_options.lns_iterations;
    const bool greedy_only_run =
        runtime_options.assignment_search_iterations <= 0 &&
        runtime_options.local_sequence_search_iterations <= 0 &&
        runtime_options.shuffle_sequence_search_iterations <= 0 &&
        improvement_iters <= 0;

    if (greedy_only_run)
    {
        return run_greedy_only_pipeline(
            argc, argv,
            runtime_options,
            instance_info,
            loadedSequence,
            relopush_single_robot_makespan,
            handoff_server.get());
    }

    // Multi-strategy search mode
    std::cout << "[Search] Evaluating greedy baseline..." << std::endl;
    AllocationScenarioPlan greedy_plan = make_identity_plan(loadedSequence.size());
    const std::uint32_t greedy_parking_seed =
        mix_seed(runtime_options.base_random_seed, 0xC0FFEE01u);

    const auto greedy_start = std::chrono::steady_clock::now();
    ExecutedScenario greedy_executed = execute_allocation_scenario(
        loadedSequence, runtime_options, greedy_plan, "greedy",
        greedy_parking_seed, false);
    const double greedy_allocation_planning_time_s =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - greedy_start)
            .count();
    AllocationRunSummary greedy_summary = greedy_executed.summary;

    std::vector<std::string> robot_names = collect_robot_names(
        loadedSequence, runtime_options);
    std::vector<RobotMeta> robot_metas = collect_robot_metas(
        loadedSequence, runtime_options);

    // Geometry export short-circuits before any search: not compatible with
    // --greedy-only mode (which returns earlier, before robot_metas exists) --
    // an accepted known scope limit.
    if (!runtime_options.export_geometry_path.empty())
    {
        std::string geometry_family = "unknown";
        int geometry_index = -1;
        parse_family_index(runtime_options.input_sequence_path, geometry_family,
                           geometry_index);
        const bool export_ok = export_instance_geometry(
            runtime_options.export_geometry_path, geometry_family, geometry_index,
            loadedSequence, robot_metas, runtime_options.geometry_export_k);
        if (export_ok)
            std::cout << "[GeometryExport] Wrote geometry to "
                      << runtime_options.export_geometry_path << std::endl;
        else
            std::cerr << "[GeometryExport] Failed to write geometry to "
                      << runtime_options.export_geometry_path << std::endl;
        return export_ok ? 0 : 1;
    }

    std::mt19937 search_rng(
        mix_seed(runtime_options.base_random_seed, 0x51A7BEEFu));
    const int lns_batch_size = std::max(1, runtime_options.lns_threads);

    // Run assignment search
    AllocationRunSummary assignment_best_feasible;
    AllocationRunSummary assignment_best_partial;
    bool assignment_has_feasible = false;
    bool assignment_has_partial = false;

    run_assignment_reassignment_search(
        loadedSequence, runtime_options, greedy_plan, greedy_summary,
        robot_names, lns_batch_size, search_rng,
        assignment_best_feasible, assignment_best_partial,
        assignment_has_feasible, assignment_has_partial);

    AllocationRunSummary assignment_report =
        assignment_has_feasible ? assignment_best_feasible : assignment_best_partial;

    // Run sequence search methods
    std::vector<SearchTrialRecord> sequence_trial_records;
    std::unordered_set<std::string> tried_sequence_signatures;
    tried_sequence_signatures.insert(task_order_signature(greedy_plan));
    LearnedOrderConstraints disabled_order_constraints =
        make_learned_order_constraints(loadedSequence.size());
    int local_sequence_enforced_constraints = 0;
    int full_shuffle_enforced_constraints = 0;
    int lns_enforced_constraints = 0;

    const std::string local_sequence_label = "local-edit-task-sequence";
    auto local_sequence_outcome = run_sequence_search_method(
        loadedSequence, runtime_options, greedy_plan, greedy_summary,
        local_sequence_label,
        "Local task-sequence search (swap/relocate edits)",
        "SeqLocal",
        runtime_options.local_sequence_search_iterations,
        mutate_sequence_plan,
        lns_batch_size, search_rng,
        tried_sequence_signatures, sequence_trial_records,
        disabled_order_constraints,
        &local_sequence_enforced_constraints);

    const std::string sequence_label = "random-task-sequence";
    auto full_shuffle_outcome = run_sequence_search_method(
        loadedSequence, runtime_options, greedy_plan, greedy_summary,
        sequence_label,
        "Randomized task-sequence search (full random shuffle)",
        "SeqFull",
        runtime_options.shuffle_sequence_search_iterations,
        mutate_full_shuffle_sequence_plan,
        lns_batch_size, search_rng,
        tried_sequence_signatures, sequence_trial_records,
        disabled_order_constraints,
        &full_shuffle_enforced_constraints);

    // Select LNS seed
    const AllocationRunSummary *lns_seed_summary = &greedy_summary;
    if (assignment_has_feasible &&
        is_better_run(assignment_best_feasible, *lns_seed_summary))
    {
        lns_seed_summary = &assignment_best_feasible;
    }
    if (local_sequence_outcome.has_feasible &&
        is_better_run(local_sequence_outcome.best_feasible, *lns_seed_summary))
    {
        lns_seed_summary = &local_sequence_outcome.best_feasible;
    }
    if (full_shuffle_outcome.has_feasible &&
        is_better_run(full_shuffle_outcome.best_feasible, *lns_seed_summary))
    {
        lns_seed_summary = &full_shuffle_outcome.best_feasible;
    }

    // Policy-gradient PoC table export (MARS/14pg-poc-design.md section 4):
    // short-circuits before the DQN/LNS search itself, but AFTER the seed
    // plan is finalized above -- unlike --export-geometry= (which runs
    // earlier and therefore cannot see the seed makespan v4 features
    // actually normalize by, only the greedy one). Not compatible with
    // --greedy-only mode (returns even earlier, before greedy_summary
    // exists) -- the same accepted scope limit --export-geometry= documents.
    if (!runtime_options.export_pg_tables_path.empty())
    {
        std::string pg_family = "unknown";
        int pg_index = -1;
        parse_family_index(runtime_options.input_sequence_path, pg_family, pg_index);
        const bool pg_export_ok = export_pg_tables(
            runtime_options.export_pg_tables_path, pg_family, pg_index,
            loadedSequence, robot_names, robot_metas,
            greedy_summary, *lns_seed_summary);
        if (pg_export_ok)
            std::cout << "[PgTableExport] Wrote PG tables to "
                      << runtime_options.export_pg_tables_path << std::endl;
        else
            std::cerr << "[PgTableExport] Failed to write PG tables to "
                      << runtime_options.export_pg_tables_path << std::endl;
        return pg_export_ok ? 0 : 1;
    }

    // Policy-gradient PoC parity validation only (script/pg/parity_check.py):
    // an independent sibling of --export-pg-tables= above (same hook point,
    // same "not compatible with --greedy-only" scope limit) -- see
    // RuntimeOptions::export_decision_time_log_path's doc comment for why
    // this is a SEPARATE flag/log from --dqn-log-transitions=
    // --dqn-relabel-executed's relabeled log.
    if (!runtime_options.export_decision_time_log_path.empty())
    {
        std::string dt_family = "unknown";
        int dt_index = -1;
        parse_family_index(runtime_options.input_sequence_path, dt_family, dt_index);
        const bool dt_export_ok = export_decision_time_log(
            runtime_options.export_decision_time_log_path, dt_family, dt_index,
            loadedSequence, robot_metas, lns_seed_summary->makespan);
        if (dt_export_ok)
            std::cout << "[DecisionTimeLog] Wrote decision-time step log to "
                      << runtime_options.export_decision_time_log_path << std::endl;
        else
            std::cerr << "[DecisionTimeLog] Failed to write decision-time step log to "
                      << runtime_options.export_decision_time_log_path << std::endl;
        return dt_export_ok ? 0 : 1;
    }

    // Run allocation-improvement search (LNS by default, or DQN online search).
    std::mt19937 lns_rng(
        mix_seed(runtime_options.base_random_seed, 0x1EA5E123u));
    const bool use_dqn_search =
        runtime_options.search_improvement_mode == SearchImprovementMode::DQN;
    auto lns_outcome = use_dqn_search
        ? run_dqn_search(
              loadedSequence, runtime_options, *lns_seed_summary,
              greedy_plan, robot_names, robot_metas, lns_batch_size, lns_rng,
              disabled_order_constraints,
              &lns_enforced_constraints)
        : run_adaptive_lns_search(
              loadedSequence, runtime_options, *lns_seed_summary,
              greedy_plan, robot_names, lns_batch_size, lns_rng,
              disabled_order_constraints,
              &lns_enforced_constraints);

    // Print order constraint learning stats
    if (runtime_options.enable_order_constraint_learning)
    {
        std::cout << "[Learn][Order] " << local_sequence_label
                  << " enforced precedence constraints: "
                  << local_sequence_enforced_constraints << std::endl;
        std::cout << "[Learn][Order] " << sequence_label
                  << " enforced precedence constraints: "
                  << full_shuffle_enforced_constraints << std::endl;
        std::cout << "[Learn][Order] " << (use_dqn_search ? "dqn-online" : "lns-adaptive")
                  << " enforced precedence constraints: "
                  << lns_enforced_constraints << std::endl;
    }
    else
    {
        std::cout << "[Learn][Order] disabled" << std::endl;
    }

    // Print comparison summary
    std::cout << "[Compare] Makespan summary" << std::endl;
    print_comparison_line(greedy_summary, greedy_summary.makespan);
    print_search_method_summary("fixed-sequence-random-reassign",
                                assignment_has_feasible,
                                assignment_best_feasible,
                                assignment_has_partial,
                                assignment_best_partial,
                                greedy_summary.makespan);
    print_search_method_summary(local_sequence_label,
                                local_sequence_outcome.has_feasible,
                                local_sequence_outcome.best_feasible,
                                local_sequence_outcome.has_partial,
                                local_sequence_outcome.best_partial,
                                greedy_summary.makespan);
    print_search_method_summary(sequence_label,
                                full_shuffle_outcome.has_feasible,
                                full_shuffle_outcome.best_feasible,
                                full_shuffle_outcome.has_partial,
                                full_shuffle_outcome.best_partial,
                                greedy_summary.makespan);
    print_search_method_summary(use_dqn_search ? "dqn-online" : "lns-adaptive",
                                lns_outcome.has_feasible,
                                lns_outcome.best_feasible,
                                lns_outcome.has_partial,
                                lns_outcome.best_partial,
                                greedy_summary.makespan);

    // Select best overall
    AllocationRunSummary local_sequence_report =
        local_sequence_outcome.has_feasible
            ? local_sequence_outcome.best_feasible
            : local_sequence_outcome.best_partial;

    AllocationRunSummary sequence_report =
        full_shuffle_outcome.has_feasible
            ? full_shuffle_outcome.best_feasible
            : full_shuffle_outcome.best_partial;

    AllocationRunSummary lns_report =
        lns_outcome.has_feasible
            ? lns_outcome.best_feasible
            : lns_outcome.best_partial;

    std::vector<AllocationRunSummary> summaries = {
        greedy_summary,
        assignment_report,
        local_sequence_report,
        sequence_report,
        lns_report};

    AllocationRunSummary best_summary = greedy_summary;
    if (assignment_has_feasible && is_better_run(assignment_best_feasible, best_summary))
        best_summary = assignment_best_feasible;
    if (local_sequence_outcome.has_feasible && is_better_run(local_sequence_outcome.best_feasible, best_summary))
        best_summary = local_sequence_outcome.best_feasible;
    if (full_shuffle_outcome.has_feasible && is_better_run(full_shuffle_outcome.best_feasible, best_summary))
        best_summary = full_shuffle_outcome.best_feasible;
    if (lns_outcome.has_feasible && is_better_run(lns_outcome.best_feasible, best_summary))
        best_summary = lns_outcome.best_feasible;

    const double lns_best_makespan =
        lns_outcome.has_feasible
            ? lns_outcome.best_feasible.makespan
            : std::numeric_limits<double>::infinity();

    const ExecutedScenario *cached_best_executed = nullptr;
    if (best_summary.label == greedy_summary.label &&
        same_execution_signature(best_summary, greedy_summary))
    {
        cached_best_executed = &greedy_executed;
    }
    if (lns_outcome.best_feasible_execution &&
        best_summary.label == lns_outcome.best_feasible.label &&
        same_execution_signature(best_summary, lns_outcome.best_feasible))
    {
        cached_best_executed = lns_outcome.best_feasible_execution.get();
    }

    // Finalize selected scenario
    return finalize_and_replay_best(
        argc, argv,
        runtime_options,
        instance_info,
        loadedSequence,
        summaries,
        best_summary,
        cached_best_executed,
        greedy_allocation_planning_time_s,
        lns_outcome.lns_batch_planning_times_s,
        lns_outcome.lns_batch_best_makespans,
        lns_outcome.lns_batch_failed_iterations,
        lns_outcome.lns_candidate_planning_times_s,
        lns_outcome.lns_candidate_feasible,
        lns_outcome.lns_candidate_makespans,
        lns_outcome.lns_failed_iterations,
        static_cast<int>(robot_names.size()),
        relopush_single_robot_makespan,
        greedy_summary.makespan,
        lns_best_makespan,
        sequence_trial_records,
        handoff_server.get());
}

#ifndef PHASTAR_PUSH_NO_MAIN
int main(int argc, char **argv)
{
    std::cout << "== Pushing ==" << std::endl;
    return phastar_push_demo_main(argc, argv);
}
#endif
