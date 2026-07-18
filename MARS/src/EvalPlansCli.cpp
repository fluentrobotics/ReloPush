#include <EvalPlansCli.h>
#include <AllocationSearch.h>
#include <PHAstarPushDemoTypes.h>
#include <ReloPush/TaskAllocation.hpp> // FinalAllocation

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{

std::size_t skip_ws(const std::string &s, std::size_t pos)
{
  while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos])))
    ++pos;
  return pos;
}

// Locates the value-start position for a top-level "key": token. Returns
// std::string::npos if `key` never appears as a quoted key immediately
// followed by (optional whitespace and) a colon.
std::size_t find_key_value_start(const std::string &s, const std::string &key)
{
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = 0;
  while (true)
  {
    pos = s.find(needle, pos);
    if (pos == std::string::npos)
      return std::string::npos;
    std::size_t after = skip_ws(s, pos + needle.size());
    if (after < s.size() && s[after] == ':')
      return skip_ws(s, after + 1);
    pos = after; // substring matched but not as an actual "key": token; keep looking
  }
}

// Parses a JSON string literal starting at `pos` (must point at the opening
// '"'). Handles \" and \\ escapes; every other character (including any
// other backslash escape) is passed through verbatim, sufficient for the
// plain instance-id strings this format uses. Returns false if malformed (no
// opening quote at `pos`, or no closing quote before the end of the string).
bool parse_json_string(const std::string &s, std::size_t pos, std::string &out,
                       std::size_t &end_pos)
{
  if (pos >= s.size() || s[pos] != '"')
    return false;
  ++pos;
  std::string result;
  while (pos < s.size() && s[pos] != '"')
  {
    if (s[pos] == '\\' && pos + 1 < s.size())
    {
      ++pos;
      result += s[pos];
    }
    else
    {
      result += s[pos];
    }
    ++pos;
  }
  if (pos >= s.size())
    return false; // no closing quote
  end_pos = pos + 1;
  out = result;
  return true;
}

// Parses a flat JSON array of non-negative integers starting at `pos` (must
// point at '['). Returns false if malformed: no opening/closing bracket, a
// non-numeric element, or a negative value.
bool parse_json_uint_array(const std::string &s, std::size_t pos,
                           std::vector<std::size_t> &out, std::size_t &end_pos)
{
  if (pos >= s.size() || s[pos] != '[')
    return false;
  ++pos;
  out.clear();
  pos = skip_ws(s, pos);
  if (pos < s.size() && s[pos] == ']')
  {
    end_pos = pos + 1;
    return true;
  }

  while (true)
  {
    pos = skip_ws(s, pos);
    const std::size_t num_start = pos;
    while (pos < s.size() &&
          (std::isdigit(static_cast<unsigned char>(s[pos])) || s[pos] == '-'))
      ++pos;
    if (pos == num_start)
      return false;

    try
    {
      std::size_t parsed_chars = 0;
      const long long value = std::stoll(s.substr(num_start, pos - num_start), &parsed_chars);
      if (parsed_chars != pos - num_start || value < 0)
        return false;
      out.push_back(static_cast<std::size_t>(value));
    }
    catch (const std::exception &)
    {
      return false;
    }

    pos = skip_ws(s, pos);
    if (pos >= s.size())
      return false;
    if (s[pos] == ',')
    {
      ++pos;
      continue;
    }
    if (s[pos] == ']')
    {
      end_pos = pos + 1;
      return true;
    }
    return false;
  }
}

} // namespace

bool parse_eval_plan_line(const std::string &line, EvalPlanRequest &out, std::string &error)
{
  const std::size_t id_pos = find_key_value_start(line, "id");
  if (id_pos == std::string::npos)
  {
    error = "missing \"id\" key";
    return false;
  }
  std::string id;
  std::size_t after_id = 0;
  if (!parse_json_string(line, id_pos, id, after_id))
  {
    error = "malformed \"id\" string";
    return false;
  }
  // Populated as soon as it's known, even if "order"/"assign" below turn out
  // to be malformed -- callers (run_eval_plans_cli) can then still report the
  // intended id on a malformed-line CSV row instead of a synthetic
  // placeholder, which is far more useful for tracking down which plan in a
  // large batch failed to parse.
  out.id = id;

  const std::size_t order_pos = find_key_value_start(line, "order");
  if (order_pos == std::string::npos)
  {
    error = "missing \"order\" key";
    return false;
  }
  std::vector<std::size_t> order;
  std::size_t after_order = 0;
  if (!parse_json_uint_array(line, order_pos, order, after_order))
  {
    error = "malformed \"order\" array (expected a flat array of non-negative integers)";
    return false;
  }

  const std::size_t assign_pos = find_key_value_start(line, "assign");
  if (assign_pos == std::string::npos)
  {
    error = "missing \"assign\" key";
    return false;
  }
  std::vector<std::size_t> assign;
  std::size_t after_assign = 0;
  if (!parse_json_uint_array(line, assign_pos, assign, after_assign))
  {
    error = "malformed \"assign\" array (expected a flat array of non-negative integers)";
    return false;
  }

  if (order.size() != assign.size())
  {
    std::ostringstream oss;
    oss << "\"order\" and \"assign\" length mismatch (" << order.size() << " vs "
        << assign.size() << ")";
    error = oss.str();
    return false;
  }

  out.id = std::move(id);
  out.order = std::move(order);
  out.assign = std::move(assign);
  return true;
}

bool validate_plan_order(const std::vector<std::size_t> &order, std::size_t task_count,
                         std::string &error)
{
  if (order.size() != task_count)
  {
    std::ostringstream oss;
    oss << "wrong length: got " << order.size() << " indices, expected " << task_count;
    error = oss.str();
    return false;
  }

  std::vector<bool> seen(task_count, false);
  for (std::size_t idx : order)
  {
    if (idx >= task_count)
    {
      std::ostringstream oss;
      oss << "out-of-range index " << idx << " (valid range is 0.." << (task_count - 1) << ")";
      error = oss.str();
      return false;
    }
    if (seen[idx])
    {
      std::ostringstream oss;
      oss << "duplicate index " << idx;
      error = oss.str();
      return false;
    }
    seen[idx] = true;
  }
  return true;
}

long parse_robot_index_from_name(const std::string &name)
{
  static const std::string kPrefix = "robot";
  if (name.size() <= kPrefix.size() || name.compare(0, kPrefix.size(), kPrefix) != 0)
    return -1;

  const std::string digits = name.substr(kPrefix.size());
  if (digits.empty() ||
      !std::all_of(digits.begin(), digits.end(),
                  [](unsigned char c) { return std::isdigit(c) != 0; }))
    return -1;

  try
  {
    const long long value = std::stoll(digits);
    if (value <= 0)
      return -1;
    return static_cast<long>(value - 1);
  }
  catch (const std::exception &)
  {
    return -1;
  }
}

std::string csv_escape_field(const std::string &s)
{
  if (s.find_first_of(",\"\n\r") == std::string::npos)
    return s;

  std::string out = "\"";
  out.reserve(s.size() + 2);
  for (char c : s)
  {
    if (c == '"')
      out += "\"\"";
    else
      out += c;
  }
  out += "\"";
  return out;
}

namespace
{

// One input line's parse/validation outcome, kept in input order so the
// output CSV mirrors the input file 1:1 (including malformed lines).
struct EvalPlanRecord
{
  std::string id;
  bool valid = false;
  std::vector<std::size_t> order;
  std::vector<std::size_t> assign; // aligned with `order`
};

} // namespace

bool run_eval_plans_cli(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const std::string &input_path,
    const std::string &output_path)
{
  std::ifstream in(input_path);
  if (!in.is_open())
  {
    std::cerr << "[EvalPlans] Failed to open input file: " << input_path << std::endl;
    return false;
  }

  const std::filesystem::path fs_out(output_path);
  const std::filesystem::path parent = fs_out.parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);

  std::ofstream out(output_path);
  if (!out.is_open())
  {
    std::cerr << "[EvalPlans] Failed to open output file: " << output_path << std::endl;
    return false;
  }

  const std::size_t task_count = loaded_sequence.size();

  std::vector<EvalPlanRecord> records;
  std::vector<ScenarioEvaluationRequest> requests;
  std::vector<std::size_t> request_to_record; // requests[k] <-> records[request_to_record[k]]

  std::string line;
  int line_no = 0;
  while (std::getline(in, line))
  {
    ++line_no;
    if (line.find_first_not_of(" \t\r\n") == std::string::npos)
      continue; // blank line: formatting noise, not a malformed entry

    EvalPlanRecord rec;
    EvalPlanRequest parsed;
    std::string issue;
    if (!parse_eval_plan_line(line, parsed, issue))
    {
      std::cerr << "[EvalPlans] Line " << line_no << ": " << issue << std::endl;
      // parsed.id may already be populated (e.g. "order"/"assign" was the
      // part that failed) -- prefer it so the malformed-line CSV row is
      // still traceable to its intended id; fall back to a synthetic
      // placeholder only when "id" itself never parsed.
      rec.id = !parsed.id.empty() ? parsed.id : "line" + std::to_string(line_no);
      records.push_back(std::move(rec));
      continue;
    }

    if (!validate_plan_order(parsed.order, task_count, issue))
    {
      std::cerr << "[EvalPlans] Line " << line_no << " (id=" << parsed.id << "): " << issue
                << std::endl;
      rec.id = parsed.id;
      records.push_back(std::move(rec));
      continue;
    }

    rec.id = parsed.id;
    rec.valid = true;
    rec.order = parsed.order;
    rec.assign = parsed.assign;

    AllocationScenarioPlan plan;
    plan.task_order = rec.order;
    plan.preferred_robot_names_by_original_task.assign(task_count, std::string());
    for (std::size_t k = 0; k < rec.order.size(); ++k)
    {
      const std::size_t task = rec.order[k];
      const std::size_t robot_idx = k < rec.assign.size() ? rec.assign[k] : 0;
      if (task < plan.preferred_robot_names_by_original_task.size())
        plan.preferred_robot_names_by_original_task[task] =
            "robot" + std::to_string(robot_idx + 1);
    }

    ScenarioEvaluationRequest req;
    req.plan = std::move(plan);
    req.label = "eval-plans";
    req.parking_seed = 1u; // fixed: oracle evaluation should be deterministic
    requests.push_back(std::move(req));
    records.push_back(std::move(rec));
    request_to_record.push_back(records.size() - 1);
  }

  const auto eval_start = std::chrono::steady_clock::now();
  auto results = evaluate_scenario_batch(loaded_sequence, options, requests);
  const double total_wall_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - eval_start).count();
  // evaluate_scenario_batch evaluates the whole input concurrently (up to
  // options.lns_threads workers), so a true per-plan wall time is not
  // separately observable; report the batch mean per plan instead (see
  // EvalPlansCli.h's doc comment on this field).
  const double per_plan_wall_s =
      requests.empty() ? 0.0 : total_wall_s / static_cast<double>(requests.size());

  std::vector<bool> has_result(records.size(), false);
  std::vector<AllocationRunSummary> summaries(records.size());
  for (std::size_t k = 0; k < results.size(); ++k)
  {
    const std::size_t rec_idx = request_to_record[k];
    has_result[rec_idx] = true;
    summaries[rec_idx] = std::move(results[k].summary);
  }

  out << "id,feasible,makespan,first_fail_step,executed_robots,eval_wall_s\n";
  out << std::fixed << std::setprecision(6);

  for (std::size_t i = 0; i < records.size(); ++i)
  {
    const EvalPlanRecord &rec = records[i];
    out << csv_escape_field(rec.id) << ",";

    if (!rec.valid || !has_result[i])
    {
      out << -1 << "," << -1 << "," << -1 << "," << "" << "," << 0 << "\n";
      continue;
    }

    const AllocationRunSummary &summary = summaries[i];
    const bool feasible = summary.all_tasks_succeeded;

    long first_fail_step = -1;
    if (!feasible)
    {
      for (std::size_t t = 0; t < summary.task_rows.size(); ++t)
      {
        if (summary.task_rows[t].status != "SUCCESS")
        {
          first_fail_step = static_cast<long>(t);
          break;
        }
      }
    }

    out << (feasible ? 1 : 0) << "," << (feasible ? summary.makespan : -1.0) << ","
        << first_fail_step << ",";

    for (std::size_t pos = 0; pos < rec.order.size(); ++pos)
    {
      if (pos > 0)
        out << ";";
      long executed_robot = -1;
      if (pos < summary.task_rows.size() && summary.task_rows[pos].status == "SUCCESS")
        executed_robot = parse_robot_index_from_name(summary.task_rows[pos].robot_name);
      out << executed_robot;
    }

    out << "," << per_plan_wall_s << "\n";
  }

  return true;
}
