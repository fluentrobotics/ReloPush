#pragma once

#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

enum class ParkingCandidateMode
{
    RANDOM,
    EXPAND,
    CONNECTED,
    CONNECTED_VFH,
    REVERSE_RECENT,
    REVERSE_RECENT_SHORTER,
};

// Method used to improve the greedy allocation. LNS keeps the existing
// destroy/repair search; DQN replaces it with the per-instance online
// Q-learning allocator (run_dqn_search).
enum class SearchImprovementMode
{
    LNS,
    DQN,
};

enum class TransitPlannerMethod
{
    PrimaryHybridAStar,
    FineHybridAStar,
    ContactBoundaryGeometricHybridAStar,
    GhostHybridAStar,
    GeometryFallbackHybridAStar,
    ReedSheppFallback,
    AllCandidateReedShepp,
    ReverseStraightEscapeFineHybridAStar,
    ReverseLeftEscapeFineHybridAStar,
    ReverseRightEscapeFineHybridAStar,
    ReverseStraightEscapeReedShepp,
    ReverseLeftEscapeReedShepp,
    ReverseRightEscapeReedShepp,
};

enum class ReferenceEGraphUse
{
    Disabled,
    EnabledWhenAvailable,
};

enum class TransitPlannerApplicability
{
    Always,
    TightOrContactOnly,
    ContactBoundaryOnly,
    StartContactOnly,
    NonStartContactOnly,
    StartContactTightOrContactOnly,
    NonStartContactTightOrContactOnly,
};

struct TransitPlannerStep
{
    TransitPlannerMethod method;
    ReferenceEGraphUse reference_egraph = ReferenceEGraphUse::Disabled;
    TransitPlannerApplicability applicability = TransitPlannerApplicability::Always;
};

inline const char *transit_planner_method_name(TransitPlannerMethod method)
{
    switch (method)
    {
    case TransitPlannerMethod::PrimaryHybridAStar:
        return "primary Hybrid A*";
    case TransitPlannerMethod::FineHybridAStar:
        return "fine Hybrid A*";
    case TransitPlannerMethod::ContactBoundaryGeometricHybridAStar:
        return "contact-boundary geometric Hybrid A*";
    case TransitPlannerMethod::GhostHybridAStar:
        return "ghost Hybrid A*";
    case TransitPlannerMethod::GeometryFallbackHybridAStar:
        return "geometry fallback Hybrid A*";
    case TransitPlannerMethod::ReedSheppFallback:
        return "Reed-Shepp fallback";
    case TransitPlannerMethod::AllCandidateReedShepp:
        return "all-candidate Reed-Shepp";
    case TransitPlannerMethod::ReverseStraightEscapeFineHybridAStar:
        return "reverse-straight escape + fine Hybrid A*";
    case TransitPlannerMethod::ReverseLeftEscapeFineHybridAStar:
        return "reverse-left escape + fine Hybrid A*";
    case TransitPlannerMethod::ReverseRightEscapeFineHybridAStar:
        return "reverse-right escape + fine Hybrid A*";
    case TransitPlannerMethod::ReverseStraightEscapeReedShepp:
        return "reverse-straight escape + Reed-Shepp";
    case TransitPlannerMethod::ReverseLeftEscapeReedShepp:
        return "reverse-left escape + Reed-Shepp";
    case TransitPlannerMethod::ReverseRightEscapeReedShepp:
        return "reverse-right escape + Reed-Shepp";
    }
    return "unknown";
}

inline const char *reference_egraph_use_name(ReferenceEGraphUse use)
{
    return use == ReferenceEGraphUse::EnabledWhenAvailable
               ? "Reference E-Graph when available"
               : "no Reference E-Graph";
}

inline const char *transit_planner_applicability_name(
    TransitPlannerApplicability applicability)
{
    switch (applicability)
    {
    case TransitPlannerApplicability::Always:
        return "always";
    case TransitPlannerApplicability::TightOrContactOnly:
        return "tight/contact only";
    case TransitPlannerApplicability::ContactBoundaryOnly:
        return "contact-boundary only";
    case TransitPlannerApplicability::StartContactOnly:
        return "start-contact only";
    case TransitPlannerApplicability::NonStartContactOnly:
        return "non-start-contact only";
    case TransitPlannerApplicability::StartContactTightOrContactOnly:
        return "start-contact tight/contact only";
    case TransitPlannerApplicability::NonStartContactTightOrContactOnly:
        return "non-start-contact tight/contact only";
    }
    return "unknown";
}

struct RuntimeOptions
{
    // visualization options
    bool debug_vis = false;
    bool visualize_relopush_plan = false;
    bool enable_visualization = true; // show results
    bool print_planning_status = true;

    // Default planning parameters
    double default_xy_resolution = 0.2;
    double default_yaw_resolution = 0.523598775598298873; // pi / 6
    double default_time_step = 2.5;
    double default_rs_step_size = 0.3;
    int default_collision_steps = 2;
    double default_collision_check_time_step = 0.05;
    double default_max_steer = 0.523598775598298873; // pi / 6
    double default_analytic_threshold_scale = 5.0;
    double default_turn_penalty = 1.25;
    double default_reverse_penalty = 10.0; // 50
    double default_switch_penalty = 1.2;
    double default_wait_penalty = 10.0;
    double default_max_time = 1000.0;
    double default_final_push_distance = 0.025;
    double default_inflation = 1.0;
    double default_safety_margin = 0.03;
    double default_robot_collision_inflation = 1.005;
    double retraction_distance = 0.11;
    int planner_expansion_threads = 1;
    int max_search_iterations = 250; // 300;
    bool robot_boundary_origin_only = true;

    // Fine segment options (for replanning)
    int fine_segment_max_search_iterations = 750; // 0
    double fine_segment_xy_resolution = 0.1;
    double fine_segment_yaw_resolution = 0.523598775598298873; // pi / 6
    double fine_segment_time_step = 1.6;
    double fine_segment_rs_step_size = 0.16;
    double fine_segment_collision_check_time_step = 0.05;
    int contact_boundary_max_search_iterations = 750; // 500
    double contact_boundary_xy_resolution = 0.08;      // 0.08
    double contact_boundary_yaw_resolution = 0.235;    // 0.26; // 0.235
    double contact_boundary_time_step = 1.0;
    double contact_boundary_rs_step_size = 0.10;
    double contact_boundary_reverse_penalty = 2.0;
    double contact_boundary_holonomic_resolution = 0.10;

    // E-Graph
    double reference_egraph_epsilon = 10.0;
    double reference_egraph_waypoint_spacing = 0.10;
    double reference_egraph_snap_radius = 0.25;
    double reference_egraph_snap_yaw = M_PI / 3.0;
    int reference_egraph_successor_lookahead = 3;
    int reference_egraph_max_nodes = 120;

    // Safe Parking
    ParkingCandidateMode parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT_SHORTER;
    int safe_parking_max_search_iterations = 25; // 0 inherits max_search_iterations
    // Iteration cap for the safe-parking connected-candidate frontier sweeps
    // (see Params::safe_parking_expand_max_iterations, which this is wired
    // to via initialize_params). Unrelated to safe_parking_max_search_iterations
    // above, which only bounds the PHAStar relocation search. Default (3000)
    // matches the previously-hardcoded kMaxExpandIterations constant, so
    // default behavior is unchanged.
    int default_safe_parking_expand_iterations = 3000;
    bool enable_failed_candidate_idle_parking = true;
    int failed_candidate_initial_transit_failure_threshold = 2;
    bool enable_order_constraint_learning = true;
    bool has_fixed_random_seed = false;
    std::uint32_t base_random_seed = 0;

    // Assignment Refinement options
    int assignment_search_iterations = 0; // for random assignment
    int local_sequence_search_iterations = 0;
    int shuffle_sequence_search_iterations = 0;
    int lns_iterations = 10;
    int lns_threads = 5;
    int robot_count = 3;                       // capped by the predefined MARS robot set
    bool enable_lns_fine_segment_retry = true; // for LNS
    bool lns_reassign_only = false;            // false: lns-task-reassign default

    // Allocation-improvement method. LNS (default) keeps the destroy/repair
    // search; DQN uses the per-instance online Q-learning allocator.
    SearchImprovementMode search_improvement_mode = SearchImprovementMode::LNS;
    double dqn_learning_rate = 0.05;
    // DQN iteration budget, separate from lns_iterations. DQN needs a warmup
    // of ~15-20 iterations before the Q-policy becomes useful.
    int dqn_iterations = 40;
    double dqn_epsilon_start = 0.3;
    double dqn_epsilon_end = 0.02;
    int dqn_grad_steps_per_iter = 64;
    int dqn_minibatch_size = 32;
    // 0 = linear Q (default); >0 = 1-hidden-layer MLP with this many units.
    int dqn_hidden_units = 0;
    // Feature-set version for the DQN allocator. 1 = legacy 7-feature set
    // (bit-identical to the original implementation). 2 = redesigned 9-dim
    // feature vector + geometric precompute + forward schedule simulation +
    // new masking rule + exploration shaping (see MARS/08dqn-feature-redesign.md).
    // 3 = explicit (task, robot) construction: the policy chooses a candidate
    // (task, robot) pair at every step (instead of task-only with earliest-
    // available-robot dispatch), scored by a trimmed 10-dim robot-conditional
    // feature vector that reuses the v2 geometry/forward-sim machinery (see
    // DqnFeaturesV2.h make_feature_vector_v3 and DqnAllocationSearch.cpp
    // construct_order_v3). 4 = same (task, robot) action space and
    // construct/ingest machinery as 3, with a 12-dim feature vector (see
    // DqnFeaturesV2.h make_feature_vector_v4) that adds two schedule-state
    // features: load_imbalance (post-commit per-robot free-time spread) and
    // idle_robot_congestion (parked-robot blockage of the candidate's own
    // corridor during its execution window).
    int dqn_feature_version = 1;
    // v2 only: fraction of the epsilon-random branch that follows the
    // reference (seed) order's next pick instead of sampling uniformly.
    double dqn_explore_ref_bias = 0.5;
    // v2 only: a learned-precedence edge only hard-masks a candidate once its
    // evidence count reaches this threshold; below it, it only feeds the
    // learned_risk feature.
    int dqn_learned_hard_evidence = 3;
    // v2 only: when non-empty, appends one CSV row per legal candidate at
    // every construction step to this path (see MARS/09rl-pretrained-study-plan.md
    // section 3.1). Empty = disabled (default).
    std::string dqn_log_transitions_path;

    // Risk-decomposed scoring (Decision 2), v3/v4 only. 0 = penalty (default):
    // the original single-model design, a single Q-head regressed onto
    // -makespan/S (feasible) or a fixed infeasibility penalty (infeasible).
    // 1 = decomposed: a second model (f_head, same class/dim/hidden as the Q
    // model) is trained as a binary failure-probability predictor, and the
    // exploit branch of construct_order_v3/v4 first filters candidates to
    // those at or below dqn_fail_threshold before taking the Q-argmax among
    // survivors (falling back to the lowest-p_fail candidate if none
    // survive) -- see DqnAllocationSearch.cpp construct_order_v3's decomposed
    // branch. Requires dqn_feature_version >= 3; parse_runtime_options
    // downgrades an inconsistent --dqn-scoring=decomposed back to penalty
    // (with a stderr warning) rather than aborting -- see
    // RuntimeOptionsParsing.cpp.
    int dqn_scoring_mode = 0;
    // Decomposed scoring only: a candidate survives the exploit-branch filter
    // iff sigmoid(f_head.predict(phi)) <= this threshold.
    double dqn_fail_threshold = 0.5;
    // Decomposed scoring only: positive-class (the fail_pos step of an
    // infeasible episode) loss weight for f_head's logistic training. 0 =
    // auto: recomputed at each training call as clamp(#neg/#pos, 1, 10) over
    // that call's f_replay buffer. >0 = fixed weight used for every call.
    double dqn_fail_pos_weight = 0.0;

    // Executed-robot relabeled corpus logging/training (v3/v4 only, requires
    // dqn_feature_version >= 3; a no-op flag otherwise). false (default):
    // ingest_rollout_v3 replays and (if --dqn-log-transitions= is set together
    // with dqn_feature_version == 2) logs against the INTENDED (constructed)
    // assignment -- byte-identical to today, and v3/v4 + --dqn-log-transitions=
    // still declines to log (unchanged). true: run_dqn_search recovers, per
    // task, the robot that ACTUALLY executed it from the run summary (same
    // row-position -> original-task-index correspondence as
    // count_assignment_divergence: executed.task_rows[i] <-> plan.task_order[i]
    // -- see AssignmentDivergence's doc comment in DqnAllocationSearch.h),
    // falling back to the intended robot for any task with no definite
    // executed robot (failing step under early abort, unexecuted tail).
    // ingest_rollout_v3_relabeled replays with commit_for against this
    // executed-or-intended sequence instead of the intended one, and, when
    // --dqn-log-transitions= is also set, writes the extended v3/v4 corpus CSV
    // (candidate_robot/executed_robot/diverged columns, 12 phi columns -- see
    // TransitionLogger::log_rollout) built from a fresh per-step (task, robot)
    // candidate enumeration against the corrected state, not the construct-time
    // step log (which reflects intended-state predictions). See
    // DqnAllocationSearch.cpp run_dqn_search's relabel branch.
    bool dqn_relabel_executed = false;

    // Fine-tune mode (see MARS/09rl-pretrained-study-plan.md section 3.4):
    // when non-empty, run_dqn_search loads pretrained weights from this path
    // via QModel::load and, on success, uses the dqn_finetune_* hyperparameters
    // below instead of the cold-start dqn_learning_rate/dqn_epsilon_* for the
    // rest of the run. Empty = disabled (cold start, default). Applies in
    // every dqn_feature_version (1-4): QModel model_dim already tracks
    // dqn_feature_version before this load happens, so this loads the correct
    // 7/9/10/12-dim weights file for whichever version is active.
    std::string dqn_init_weights_path;
    // Fine-tune learning rate: 1/5 of the cold-start dqn_learning_rate default
    // (0.05), since a pretrained model already has useful weights and only
    // needs small adjustments rather than starting from scratch.
    double dqn_finetune_learning_rate = 0.01;
    double dqn_finetune_epsilon_start = 0.1;
    double dqn_finetune_epsilon_end = 0.02;
    // Decomposed scoring only (dqn_scoring_mode == 1): when non-empty,
    // run_dqn_search loads pretrained weights for f_head (the failure-
    // probability model) from this path via QModel::load, mirroring
    // dqn_init_weights_path's handling for q_head -- on failure (missing file
    // or dim/hidden mismatch), QModel::load leaves f_head's freshly
    // initialized weights untouched and a warning is printed. Empty = f_head
    // starts from fresh (random) weights (default).
    std::string dqn_init_fail_weights_path;
    // Zero-shot evaluation mode: when true, run_dqn_search skips every
    // train_model/train_model_logistic call (both the warm-start call and the
    // once-per-batch calls at the end of the episode loop), for both q_head
    // and f_head. Replay-buffer ingestion and corpus logging still proceed
    // normally -- only the gradient updates are skipped. Intended for
    // evaluating loaded weights (dqn_init_weights_path /
    // dqn_init_fail_weights_path) with no further learning. Default false
    // (train normally).
    bool dqn_freeze_model = false;

    // When non-empty, skip search entirely and dump per-instance geometry
    // (task paths, poses, workspace, robots) as JSON to this path (see
    // MARS/09rl-pretrained-study-plan.md section 3.2). Empty = disabled.
    std::string export_geometry_path;
    // Number of arclength-resampled points per task reference path in the
    // geometry export.
    int geometry_export_k = 16;

    // Policy-gradient PoC (see MARS/14pg-poc-design.md section 4 and 9,
    // "P0"): when non-empty, skip search entirely (after the seed plan used
    // by the DQN/LNS search is selected -- see PHAstar_push_demo.cpp) and
    // dump everything script/pg/rollout_sim.py needs to reproduce
    // make_feature_vector_v4 purely from table lookups (no Reed-Shepp/
    // geometry of its own) as one JSON file: static per-task geometry/DAG,
    // the reference-order-consistent pairwise congestion matrices, and two
    // precomputed tables (transit_time, parked_blockage) over the finite set
    // of poses a robot can occupy mid-construction (its own initial pose, or
    // some task's exit pose). See PgTableExport.h. Empty = disabled
    // (default).
    std::string export_pg_tables_path;

    // Policy-gradient PoC batch-oracle CLI (see MARS/14pg-poc-design.md
    // section 4, "Batch oracle CLI"): when export_eval_plans_path is
    // non-empty, skip search entirely and evaluate every plan listed in that
    // JSONL file (one {"id","order","assign"} object per line) through the
    // same evaluate_scenario_batch() pathway the DQN/LNS search uses
    // (early-abort per early_abort_eval_on_failure, default on), writing one
    // CSV row per input line to eval_plans_out_path. See EvalPlansCli.h.
    // Empty = disabled (default).
    std::string eval_plans_path;
    // Output CSV path for --eval-plans=; required (and validated at the
    // call site) whenever eval_plans_path is non-empty.
    std::string eval_plans_out_path;

    // Stage 1 planner timing instrumentation (see MARS/include/
    // PHAstarPushDemoTypes.h's PlanTimingStats): when non-empty, after
    // evaluating the --eval-plans= batch, run_eval_plans_cli additionally
    // writes one CSV row per evaluated plan (same id/order as the main
    // output CSV) with every PlanTimingStats field, to this path. The
    // underlying PlanTimingStats accumulation itself always happens inside
    // evaluate_scenario_batch() (cheap: chrono + counters only) regardless
    // of this flag; only the extra CSV write is gated by it. Empty =
    // disabled (default; no extra file written).
    std::string eval_plans_timing_out_path;

    // Save-and-replay support (see MARS/16save-replay-implementation.md and
    // MARS/include/ExecutedScenarioSerialization.h): when either of the two
    // paths below is non-empty, --eval-plans= additionally serializes each
    // plan's full ExecutedScenario (params + entities + timetable, not just
    // its AllocationRunSummary) to a base64 blob WHILE it is still alive, in
    // evaluate_scenario_batch() -- see ScenarioEvaluationResult::
    // serialized_result. This lets a scored plan be visualized later via
    // --play-result= with no re-run of the allocation search. Empty (both,
    // default) = do not serialize, no perf regression.
    //
    // Saves only the single BEST feasible plan (see run_eval_plans_cli's
    // best-selection rule: max all_tasks_succeeded, min makespan, ties break
    // by first occurrence in input order) to this one file.
    std::string eval_plans_result_out_path;
    // Saves EVERY plan (feasible or not) to "<dir>/<id>.scn.b64", one file
    // per input line's "id". The directory is created if missing.
    std::string eval_plans_result_out_dir;

    // Save-and-replay playback (see MARS/16save-replay-implementation.md):
    // when non-empty, skip search (and even sequence loading -- see
    // PHAstar_push_demo.cpp) entirely, load a previously-saved
    // ExecutedScenario from this file via deserialize_executed_scenario_b64,
    // and hand it straight to show_results(). Empty = disabled (default).
    std::string play_result_path;

    // Policy-gradient PoC parity validation only (see
    // script/pg/parity_check.py): when non-empty, skip search entirely and
    // dump a DECISION-TIME step-candidate log (the exact phi values
    // construct_order_v3 itself scores candidates against during
    // construction, via its own step_logs parameter -- see
    // DqnAllocationSearch.cpp) to this path, using an all-zero-weight linear
    // model and epsilon=0.0 so the constructed (order, assignment) is fully
    // deterministic (every step's argmax ties at 0.0 and keeps the
    // first-seen candidate: lowest legal task index, then lowest robot
    // index -- no RNG-dependent choice is ever made). This is independent of
    // (and answers a different question than) --export-pg-tables=: that
    // exporter's ref_order/ref_assign and this log's own construction both
    // describe the SAME instance, but --dqn-log-transitions=
    // --dqn-relabel-executed logs a RELABELED trajectory (ForwardSim state
    // advanced by the EXECUTED robot at every step, which can differ from
    // the intended one on divergence -- see ingest_rollout_v3_relabeled),
    // while this logs the true DECISION-TIME trajectory (ForwardSim state
    // advanced by construct_order_v3's own chosen_robot at every step,
    // exactly matching what a real construction/rollout -- including a
    // future Python-side sampled one -- would see). Written via
    // TransitionLogger's existing extended v3/v4 schema (same columns as
    // --dqn-log-transitions= produces in relabel mode), so
    // script/pg/parity_check.py's existing CSV-reading code reads this with
    // no format changes. Empty = disabled (default).
    std::string export_decision_time_log_path;

    // When non-empty, skip search entirely and execute this single, explicitly
    // given task order (a comma-separated list of task indices, e.g.
    // "0,1,2,3") through the real multi-robot executor, reporting its true
    // makespan/feasibility. Used by external pipelines (e.g. a trained model)
    // that construct a task order and need ground-truth evaluation rather than
    // a proxy. Empty = disabled (default).
    std::string fixed_order_path_or_list;

    // During allocation search, abort a candidate plan's evaluation as soon as
    // one task fails (the plan is already infeasible, so planning the remaining
    // tasks only wastes time on fallback transit searches). Does not affect the
    // final best-scenario replay, which always runs a full execution.
    bool early_abort_eval_on_failure = true;

    // Stage 3 planner optimization (opt-in; see MARS/src/PlanningHelpers.cpp's
    // Stage 3 design note and PlanTimingStats's n_triage_*/n_gate_* fields).
    // Both default OFF: with both false, plan_initial_transit's method loop
    // and replan_transit_segment's cascade are bit-identical to Stage 1/2.
    //
    // FAILURE TRIAGE: after a tier fails, applies (in order) a BLOCKED_BY_ROBOT
    // short-circuit (only where the cascade can otherwise escalate past a
    // robot-blocked backup path -- see replan_transit_segment), a primary-tier
    // cap-hit-with-progress retry (doubled iteration cap, once), and a
    // near-goal-collision skip-Fine-go-to-Contact rule.
    bool enable_tier_triage = false;
    // HOLONOMIC FEASIBILITY PRE-GATE: before launching Fine (and again before
    // Contact), runs a cheap static-obstacles + inscribed-radius-footprint
    // reachability check at that tier's own holonomic resolution and skips
    // the tier outright when the goal is PROVABLY unreachable.
    bool enable_tier_gate = false;

    std::vector<TransitPlannerStep> initial_transit_methods = {
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
    };

    std::vector<TransitPlannerStep> segment_transit_methods = {
        /*
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::StartContactOnly},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::StartContactTightOrContactOnly},
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::NonStartContactOnly},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::NonStartContactTightOrContactOnly},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::ContactBoundaryOnly},
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::StartContactOnly},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::StartContactTightOrContactOnly},
        {TransitPlannerMethod::AllCandidateReedShepp,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::Always},
         */
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
    };

    // for figure generation
    bool enable_result_summary_figure = false;
    std::string result_summary_output_path;
    double result_summary_subplot_gap = 36.0;
    std::vector<std::string> robot_trace_colors = {
        "#70A288",
        "#DAB785",
        "#D5896F",
        "#CC79A7",
        "#E69F00",
        "#56B4E9",
    };

    // ZeroMQ
    bool integrated_mode = false;
    std::string handoff_endpoint = "tcp://127.0.0.1:5566";
    std::string input_sequence_path;
    bool run_on_robots = false;
    int robot_controller_port_start = 11110;
    int mpc_vesc_port_start = 3160;
    int mpc_localization_port_start = 3260;
    std::string mpc_vesc_ip = "127.0.0.1";
    std::string mpc_localization_ip = "127.0.0.1";
    bool spawn_mpc = true;

    // Sim-viz handoff gate (see MARS/include/SimVizHandoff.h): when true,
    // a successful run's
    // finalization step hands the ExecutedScenario off to a separately
    // running mars_sim_viz process instead of driving robots/simulators
    // in-process via run_on_robots_pipeline. Mutually exclusive with
    // run_on_robots at the two SearchOrchestrator.cpp call sites (this flag
    // wins when both are set); default false keeps existing --run-on-robots
    // behavior bit-identical.
    bool sim_viz_handoff = false;
    // Control-socket endpoint mars_sim_viz's REP server binds (PING/EXECUTE
    // protocol).
    std::string sim_viz_endpoint = "tcp://127.0.0.1:5601";
    // Directory the handoff-gate writes "<label>.scn.b64" into before
    // notifying the visualizer. Created if missing. Relative paths resolve
    // against the working directory the process was launched from (same
    // convention as --eval-plans-result-out-dir=).
    std::string sim_handoff_out_dir = "results/sim_handoff";
};
