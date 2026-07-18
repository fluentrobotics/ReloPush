#ifndef PG_TABLE_EXPORT_H
#define PG_TABLE_EXPORT_H

#include <PHAstar/Entities.h>     // RobotMeta
#include <PHAstarPushDemoTypes.h> // AllocationRunSummary

#include <string>
#include <vector>

// ==========================================
// Per-instance PG (policy-gradient) table exporter (see
// MARS/14pg-poc-design.md section 4, "Python rollout simulator", and the P0
// build item). Sibling to GeometryExport.h/--export-geometry= rather than an
// extension of it: that exporter runs BEFORE the seed plan used to normalize
// v4 features is known (see PHAstar_push_demo.cpp), so this one hooks in
// later, once the seed plan is selected, and exports a strict superset of
// static geometry PLUS two precomputed lookup tables that let
// script/pg/rollout_sim.py reproduce every one of DqnFeaturesV2.h's
// make_feature_vector_v4 dimensions via table lookups only -- no Reed-Shepp
// or corridor-blockage geometry of its own, and hence no risk of Python/C++
// geometry drift (only feature-formula drift, caught by parity_check.py).
//
// The trick that keeps Python geometry-free: in a constructive rollout, a
// robot's pose is always EITHER its own initial pose OR some task's exit
// pose (ForwardSim only ever assigns sim.pose[r] = geom.exit on commit). So
// the finite pose set P = {robot_init_1..R} U {task_exit_1..N} (size R+N,
// robot inits first, robot index order, then task exits, task index order)
// is enough to precompute:
//   - transit_time[p][t]: Reeds-Shepp length from pose P[p] to task t's
//     approach pose, divided by the same speed_transit ForwardSim::
//     preview_for divides by (the v4 "transit" feature's tau_transit term).
//   - parked_blockage[p][t]: blockage_weight(P[p], task t's combined
//     push+transit corridor, block_dist, workspace) -- exactly the quantity
//     make_feature_vector_v4's idle_robot_congestion feature (dim 11)
//     consumes for every OTHER robot parked at pose P[p] during candidate
//     task t's execution window.
// Every other v4 dimension (dmk_plus, slack, rel_avail, cong_static,
// cong_dynamic, boundary_risk, dag_violation, load_imbalance) is pure
// schedule-state arithmetic once transit time is known, so no further table
// is needed; learned_risk is always 0 in this pretraining export (see
// MARS/14pg-poc-design.md's PoC scope -- no learned order constraints).
// ==========================================

struct FinalAllocation; // forward declaration; only export_pg_tables() needs
                        // the full type

// Writes one JSON file to `out_path` (creating parent directories as
// needed), with the following top-level keys:
//   family, index, task_count, robot_count, robot_names,
//   workspace {xMin,xMax,yMin,yMax}, robot_width, block_dist, speed_transit,
//   robot_wheel_base, robot_min_turning_radius_transit,
//   greedy_makespan, seed_makespan (the S the v4 features actually
//   normalize by -- seed_summary.makespan, which is only guaranteed equal to
//   greedy_makespan when the assignment/local-sequence/full-shuffle upstream
//   searches found nothing better than greedy),
//   tasks: [{task_id, tau_fixed, approach{x,y,yaw}, exit{x,y,yaw},
//            obj_start{x,y,yaw}, obj_goal{x,y,yaw}, boundary_risk,
//            hard_pred[], soft_pred[]}, ...] (DqnV2::PairwiseGeometry's
//            per-task scalars/DAG, one entry per task index),
//   corr_overlap[a][j], w_start[j][a], w_goal[j][a] (n x n; SAME indexing
//     convention as DqnV2::PairwiseGeometry -- see make_feature_vector_v4's
//     cong_static/cong_dynamic sums for exactly how each is walked),
//   pose_count, poses: [{kind:"robot_init"|"task_exit", robot|task, x,y,yaw}]
//     (size robot_count+task_count, robot inits first in robot index order,
//     then task exits in task index order -- this ordering IS the p index
//     transit_time/parked_blockage are keyed by),
//   transit_time[p][t], parked_blockage[p][t] ((robot_count+task_count) x
//     task_count -- see the pose-set trick above),
//   ref_order, ref_assign: the certified reference (BC) trajectory recovered
//     from `seed_summary` (the same seed plan run_dqn_search warm-starts
//     from) -- ref_order is seed_summary.plan's normalized task order;
//     ref_assign[k] is the robot index (0-based) that ACTUALLY EXECUTED
//     ref_order[k] in seed_summary (via assignment_from_summary), i.e.
//     ref_assign is aligned with ref_order exactly like the batch-oracle
//     CLI's "assign" field is aligned with "order" -- see EvalPlansCli.h.
// All floating-point fields are written at %.17g-equivalent precision
// (std::setprecision(17), general/non-fixed format) for exact
// reproducibility. Returns false (logging to std::cerr) if loaded_sequence/
// robot_metas/robot_names is empty or out_path can't be opened for writing.
bool export_pg_tables(
    const std::string &out_path,
    const std::string &family, int index,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<std::string> &robot_names,
    const std::vector<RobotMeta> &robot_metas,
    const AllocationRunSummary &greedy_summary,
    const AllocationRunSummary &seed_summary);

// Policy-gradient PoC parity validation only (see
// RuntimeOptions::export_decision_time_log_path's doc comment in
// PHAstarPushDemoOptions.h for the full rationale, and
// script/pg/parity_check.py for the Python side). Builds the same per-task
// geometry/DAG as export_pg_tables (independently -- this function does not
// share state with it), constructs ONE (order, assignment) via
// construct_order_v3 with an all-zero-weight linear QModel and epsilon=0.0
// (fully deterministic: every candidate scores 0.0, so the strict-`>`
// argmax keeps the first-seen candidate at each step -- lowest legal task
// index, then lowest robot index), and writes its step_logs (every legal
// (task, robot) candidate's 12-dim phi at every step, scored against
// construct_order_v3's own DECISION-TIME ForwardSim state -- i.e. advanced
// by the CONSTRUCTED/intended robot at each step, not any executed one)
// via TransitionLogger's existing extended v3/v4 schema. `seed_makespan`
// should be the same value the paired --export-pg-tables= run for this
// instance reports (its "seed_makespan" field), so phi values normalize by
// the same S on both sides. Returns false (logging to std::cerr) on the
// same failure conditions as export_pg_tables.
bool export_decision_time_log(
    const std::string &out_path,
    const std::string &family, int index,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<RobotMeta> &robot_metas,
    double seed_makespan);

#endif // PG_TABLE_EXPORT_H
