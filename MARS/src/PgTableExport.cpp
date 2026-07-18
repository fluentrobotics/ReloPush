#include <PgTableExport.h>
#include <DqnFeaturesV2.h>
#include <DqnAllocationSearch.h> // assignment_from_summary
#include <AllocationSearch.h>    // normalized_task_order
#include <GeometryExport.h>      // json_escape_string
#include <ReloPush/TaskAllocation.hpp> // FinalAllocation

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <system_error>

namespace
{

// Writes an n x m matrix of doubles as a JSON array of arrays, 4-space
// indented, with a trailing comma after the closing "]" (every call site
// here has more keys following).
void write_matrix(std::ofstream &out, const char *name,
                  const std::vector<std::vector<double>> &m)
{
  out << "  \"" << name << "\": [\n";
  for (std::size_t a = 0; a < m.size(); ++a)
  {
    out << "    [";
    for (std::size_t b = 0; b < m[a].size(); ++b)
    {
      if (b > 0)
        out << ", ";
      out << m[a][b];
    }
    out << "]" << (a + 1 < m.size() ? "," : "") << "\n";
  }
  out << "  ],\n";
}

void write_size_t_array(std::ofstream &out, const std::vector<std::size_t> &v)
{
  out << "[";
  for (std::size_t k = 0; k < v.size(); ++k)
  {
    if (k > 0)
      out << ", ";
    out << v[k];
  }
  out << "]";
}

void write_pose(std::ofstream &out, const ReloPush::State &p)
{
  out << "{\"x\": " << p.x << ", \"y\": " << p.y << ", \"yaw\": " << p.yaw << "}";
}

} // namespace

bool export_pg_tables(
    const std::string &out_path,
    const std::string &family, int index,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<std::string> &robot_names,
    const std::vector<RobotMeta> &robot_metas,
    const AllocationRunSummary &greedy_summary,
    const AllocationRunSummary &seed_summary)
{
  if (loaded_sequence.empty() || robot_metas.empty() || robot_names.empty())
  {
    std::cerr << "[PgTableExport] Empty loaded_sequence/robot_metas/robot_names; "
                 "nothing to export."
              << std::endl;
    return false;
  }

  const std::filesystem::path fs_path(out_path);
  const std::filesystem::path parent = fs_path.parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);

  std::ofstream out(out_path);
  if (!out.is_open())
  {
    std::cerr << "[PgTableExport] Failed to open output file: " << out_path << std::endl;
    return false;
  }
  out << std::setprecision(17);

  const std::size_t n = loaded_sequence.size();
  const std::size_t robot_count = robot_metas.size();

  const auto &boundary = loaded_sequence[0].snapshot.parameters.boundary;
  const DqnV2::WorkspaceBounds bounds{boundary.xMin, boundary.xMax, boundary.yMin, boundary.yMax};
  const double robot_width = robot_metas[0].size.width;
  const double block_dist = 0.5 * robot_width + DqnV2::kObjHalfDiag;
  const double speed_transit = robot_metas[0].speed_transit;
  const double maxc = 1.0 / std::max(robot_metas[0].min_turning_radius_transit, 1e-6);
  const double wheel_base = robot_metas[0].wheel_base;

  const auto geoms = DqnV2::extract_task_geoms(loaded_sequence, robot_metas[0]);
  const auto geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);

  // Pose set P: robot inits [0,R) in robot index order, then task exits
  // [R,R+N) in task index order -- see PgTableExport.h's doc comment.
  std::vector<ReloPush::State> poses;
  poses.reserve(robot_count + n);
  for (std::size_t r = 0; r < robot_count; ++r)
    poses.emplace_back(robot_metas[r].initial_pose.x, robot_metas[r].initial_pose.y,
                       robot_metas[r].initial_pose.yaw);
  for (std::size_t t = 0; t < n; ++t)
    poses.push_back(geoms[t].exit);

  // Combined push+transit corridor per task -- same concatenation
  // make_feature_vector_v4's idle_robot_congestion term builds inline.
  std::vector<std::vector<ReloPush::State>> corridors(n);
  for (std::size_t t = 0; t < n; ++t)
  {
    corridors[t].reserve(geoms[t].push_pts.size() + geoms[t].transit_pts.size());
    corridors[t].insert(corridors[t].end(), geoms[t].push_pts.begin(), geoms[t].push_pts.end());
    corridors[t].insert(corridors[t].end(), geoms[t].transit_pts.begin(), geoms[t].transit_pts.end());
  }

  std::vector<std::vector<double>> transit_time(poses.size(), std::vector<double>(n, 0.0));
  std::vector<std::vector<double>> parked_blockage(poses.size(), std::vector<double>(n, 0.0));
  for (std::size_t p = 0; p < poses.size(); ++p)
  {
    for (std::size_t t = 0; t < n; ++t)
    {
      const double len =
          DqnV2::reeds_shepp_length(poses[p], geoms[t].approach, maxc, 0.5, wheel_base);
      transit_time[p][t] = speed_transit > 1e-9 ? len / speed_transit : 0.0;
      parked_blockage[p][t] = DqnV2::blockage_weight(poses[p], corridors[t], block_dist, bounds);
    }
  }

  // Reference (BC) trajectory: seed_summary's normalized task order, paired
  // with the robot that ACTUALLY EXECUTED each task in that run -- aligned
  // with ref_order exactly like EvalPlanRequest::assign is aligned with
  // EvalPlanRequest::order (see EvalPlansCli.h).
  const auto ref_order = normalized_task_order(seed_summary.plan, n);
  const auto ref_assign_by_task = assignment_from_summary(seed_summary, robot_names, n);
  std::vector<std::size_t> ref_assign(ref_order.size(), 0);
  for (std::size_t k = 0; k < ref_order.size(); ++k)
    ref_assign[k] = ref_order[k] < ref_assign_by_task.size() ? ref_assign_by_task[ref_order[k]] : 0;

  const double greedy_makespan =
      std::isfinite(greedy_summary.makespan) ? greedy_summary.makespan : -1.0;
  const double seed_makespan =
      std::isfinite(seed_summary.makespan) ? seed_summary.makespan : -1.0;

  out << "{\n";
  out << "  \"family\": \"" << json_escape_string(family) << "\",\n";
  out << "  \"index\": " << index << ",\n";
  out << "  \"task_count\": " << n << ",\n";
  out << "  \"robot_count\": " << robot_count << ",\n";

  out << "  \"robot_names\": [";
  for (std::size_t r = 0; r < robot_names.size(); ++r)
  {
    if (r > 0)
      out << ", ";
    out << "\"" << json_escape_string(robot_names[r]) << "\"";
  }
  out << "],\n";

  out << "  \"workspace\": {\"xMin\": " << bounds.xMin << ", \"xMax\": " << bounds.xMax
      << ", \"yMin\": " << bounds.yMin << ", \"yMax\": " << bounds.yMax << "},\n";
  out << "  \"robot_width\": " << robot_width << ",\n";
  out << "  \"block_dist\": " << block_dist << ",\n";
  out << "  \"speed_transit\": " << speed_transit << ",\n";
  out << "  \"robot_wheel_base\": " << wheel_base << ",\n";
  out << "  \"robot_min_turning_radius_transit\": " << robot_metas[0].min_turning_radius_transit
      << ",\n";
  out << "  \"greedy_makespan\": " << greedy_makespan << ",\n";
  out << "  \"seed_makespan\": " << seed_makespan << ",\n";

  out << "  \"tasks\": [\n";
  for (std::size_t t = 0; t < n; ++t)
  {
    out << "    {\n";
    out << "      \"task_id\": " << t << ",\n";
    out << "      \"tau_fixed\": " << geoms[t].tau_fixed << ",\n";
    out << "      \"approach\": ";
    write_pose(out, geoms[t].approach);
    out << ",\n";
    out << "      \"exit\": ";
    write_pose(out, geoms[t].exit);
    out << ",\n";
    out << "      \"obj_start\": ";
    write_pose(out, geoms[t].obj_start);
    out << ",\n";
    out << "      \"obj_goal\": ";
    write_pose(out, geoms[t].obj_goal);
    out << ",\n";
    out << "      \"boundary_risk\": " << geometry.boundary_risk[t] << ",\n";
    out << "      \"hard_pred\": ";
    write_size_t_array(out, geometry.hard_pred[t]);
    out << ",\n";
    out << "      \"soft_pred\": ";
    write_size_t_array(out, geometry.soft_pred[t]);
    out << "\n";
    out << "    }" << (t + 1 < n ? "," : "") << "\n";
  }
  out << "  ],\n";

  write_matrix(out, "corr_overlap", geometry.corr_overlap);
  write_matrix(out, "w_start", geometry.w_start);
  write_matrix(out, "w_goal", geometry.w_goal);

  out << "  \"pose_count\": " << poses.size() << ",\n";
  out << "  \"poses\": [\n";
  for (std::size_t p = 0; p < poses.size(); ++p)
  {
    const bool is_robot = p < robot_count;
    out << "    {\"kind\": \"" << (is_robot ? "robot_init" : "task_exit") << "\", \""
        << (is_robot ? "robot" : "task") << "\": " << (is_robot ? p : (p - robot_count))
        << ", \"x\": " << poses[p].x << ", \"y\": " << poses[p].y
        << ", \"yaw\": " << poses[p].yaw << "}" << (p + 1 < poses.size() ? "," : "") << "\n";
  }
  out << "  ],\n";

  write_matrix(out, "transit_time", transit_time);
  write_matrix(out, "parked_blockage", parked_blockage);

  out << "  \"ref_order\": ";
  write_size_t_array(out, ref_order);
  out << ",\n";
  out << "  \"ref_assign\": ";
  write_size_t_array(out, ref_assign);
  out << "\n";

  out << "}\n";
  return true;
}

bool export_decision_time_log(
    const std::string &out_path,
    const std::string &family, int index,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<RobotMeta> &robot_metas,
    double seed_makespan)
{
  if (loaded_sequence.empty() || robot_metas.empty())
  {
    std::cerr << "[DecisionTimeLog] Empty loaded_sequence/robot_metas; nothing to export."
              << std::endl;
    return false;
  }

  const std::size_t n = loaded_sequence.size();
  const auto &boundary = loaded_sequence[0].snapshot.parameters.boundary;
  const DqnV2::WorkspaceBounds bounds{boundary.xMin, boundary.xMax, boundary.yMin, boundary.yMax};
  const double robot_width = robot_metas[0].size.width;

  const auto geoms = DqnV2::extract_task_geoms(loaded_sequence, robot_metas[0]);
  const auto geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);

  // All-zero-weight linear model (QModel's hidden==0 constructor draws
  // nothing from rng and zero-inits `w`) + epsilon=0.0: every candidate's
  // predict() is exactly 0.0, so construct_order_v3's strict `>` argmax never
  // replaces the first-seen candidate -- the construction is therefore fully
  // deterministic (lowest legal task index, then lowest robot index, at
  // every step) and independent of the rng seed below (epsilon=0.0 makes the
  // epsilon-branch a probability-zero event; the per-step coin(rng) draw is
  // still consumed, exactly as construct_order_v3 always does, but its value
  // never crosses the 0.0 threshold).
  std::mt19937 rng(0xDEC1DE71u);
  QModel model(DqnV2::kFeatDimV4, 0, rng);

  RuntimeOptions options;
  options.dqn_feature_version = 4; // selects make_feature_vector_v4 inside construct_order_v3
  const LearnedOrderConstraints no_learned_constraints; // enforced.size()==0 -> no learned masking

  std::vector<std::vector<StepCandidateEntry>> step_logs;
  const TaskOrderAssignment oa = construct_order_v3(
      model, n, geoms, geometry, no_learned_constraints, robot_metas, seed_makespan,
      /*epsilon=*/0.0, options, rng, &step_logs, /*f_head=*/nullptr);
  (void)oa; // the (order, assignment) is recoverable from step_logs' chosen rows

  std::vector<StepCandidateEntry> flattened;
  for (auto &step_entries : step_logs)
    for (auto &entry : step_entries)
      flattened.push_back(std::move(entry));

  // Placeholder outcome: this log is read only for its phi/candidate_task/
  // candidate_robot/chosen columns (decision-time feature parity), never for
  // feasibility/makespan, since no real evaluation happens here.
  RolloutOutcome placeholder_outcome;
  placeholder_outcome.feasible = true;
  placeholder_outcome.makespan = seed_makespan;
  placeholder_outcome.return_target = 0.0;
  placeholder_outcome.first_fail_rank = n;
  placeholder_outcome.first_failed_task = -1;

  const std::filesystem::path fs_path(out_path);
  const std::filesystem::path parent = fs_path.parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);

  // Fresh file every call: TransitionLogger appends, and a stale prior run's
  // rows would otherwise corrupt the single-rollout assumption
  // parity_check.py's log reader makes.
  std::error_code remove_ec;
  std::filesystem::remove(fs_path, remove_ec);

  TransitionLogger logger(out_path, /*extended_v3_schema=*/true);
  logger.log_rollout(family, index, /*seed=*/0u, /*iteration=*/0, flattened, placeholder_outcome);

  std::ifstream check(out_path);
  if (!check.is_open())
  {
    std::cerr << "[DecisionTimeLog] Failed to open output file: " << out_path << std::endl;
    return false;
  }
  return true;
}
