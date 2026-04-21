#define PHASTAR_PUSH_NO_MAIN
#include "PHAstar_push_demo.cpp"

namespace
{
  std::string trim_copy(const std::string &text)
  {
    std::size_t begin = 0;
    while (begin < text.size() &&
           std::isspace(static_cast<unsigned char>(text[begin])))
    {
      ++begin;
    }

    std::size_t end = text.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(text[end - 1])))
    {
      --end;
    }

    return text.substr(begin, end - begin);
  }

  std::vector<std::string> parse_csv_row(const std::string &line)
  {
    std::vector<std::string> cells;
    std::string current;
    bool in_quotes = false;

    for (std::size_t i = 0; i < line.size(); ++i)
    {
      char c = line[i];
      if (c == '"')
      {
        if (in_quotes && i + 1 < line.size() && line[i + 1] == '"')
        {
          current.push_back('"');
          ++i;
        }
        else
        {
          in_quotes = !in_quotes;
        }
        continue;
      }

      if (c == ',' && !in_quotes)
      {
        cells.push_back(current);
        current.clear();
        continue;
      }

      current.push_back(c);
    }

    cells.push_back(current);
    return cells;
  }

  std::vector<std::string> split_order_list(const std::string &order_text)
  {
    std::vector<std::string> items;
    std::stringstream ss(order_text);
    std::string item;
    while (std::getline(ss, item, ','))
    {
      item = trim_copy(item);
      if (!item.empty())
        items.push_back(item);
    }
    return items;
  }

  std::string parse_sequence_csv_path(int argc, char **argv)
  {
    std::string default_path =
        std::string(CMAKE_SOURCE_DIR) + "/task_execution_log_lns_adaptive.csv";

    for (int i = 1; i < argc; ++i)
    {
      std::string arg = argv[i] ? argv[i] : "";
      if (arg.rfind("--sequence-csv=", 0) == 0)
      {
        return arg.substr(std::string("--sequence-csv=").size());
      }
    }

    return default_path;
  }

  int parse_trial_iteration(int argc, char **argv)
  {
    for (int i = 1; i < argc; ++i)
    {
      std::string arg = argv[i] ? argv[i] : "";
      if (arg.rfind("--trial-iteration=", 0) == 0)
      {
        std::string value = arg.substr(std::string("--trial-iteration=").size());
        try
        {
          return std::max(1, std::stoi(value));
        }
        catch (...)
        {
          std::cerr << "[Replay] Invalid --trial-iteration value: " << value
                    << std::endl;
          return -1;
        }
      }
    }

    return 0;
  }

  bool load_object_order_from_csv(const std::string &csv_path,
                                  int trial_iteration,
                                  std::vector<std::string> &object_order)
  {
    std::ifstream ifs(csv_path);
    if (!ifs.is_open())
    {
      std::cerr << "[Replay] Failed to open CSV: " << csv_path << std::endl;
      return false;
    }

    std::string header_line;
    if (!std::getline(ifs, header_line))
    {
      std::cerr << "[Replay] CSV is empty: " << csv_path << std::endl;
      return false;
    }

    std::vector<std::string> headers = parse_csv_row(header_line);
    std::unordered_map<std::string, std::size_t> column_index;
    for (std::size_t i = 0; i < headers.size(); ++i)
    {
      column_index[trim_copy(headers[i])] = i;
    }

    auto order_it = column_index.find("order");
    if (order_it != column_index.end())
    {
      std::size_t order_col = order_it->second;
      std::size_t iter_col = column_index.count("iteration") ? column_index["iteration"] : static_cast<std::size_t>(-1);

      std::string line;
      while (std::getline(ifs, line))
      {
        if (trim_copy(line).empty())
          continue;

        auto row = parse_csv_row(line);
        if (order_col >= row.size())
          continue;

        if (trial_iteration > 0 && iter_col != static_cast<std::size_t>(-1))
        {
          if (iter_col >= row.size())
            continue;
          if (std::stoi(trim_copy(row[iter_col])) != trial_iteration)
            continue;
        }

        object_order = split_order_list(row[order_col]);
        if (!object_order.empty())
          return true;
      }

      std::cerr << "[Replay] Failed to find a matching order row in: " << csv_path
                << std::endl;
      return false;
    }

    auto object_it = column_index.find("object");
    if (object_it == column_index.end())
    {
      std::cerr << "[Replay] CSV does not contain either 'order' or 'object' columns: "
                << csv_path << std::endl;
      return false;
    }

    std::size_t object_col = object_it->second;
    std::string line;
    while (std::getline(ifs, line))
    {
      if (trim_copy(line).empty())
        continue;

      auto row = parse_csv_row(line);
      if (object_col >= row.size())
        continue;

      std::string object_name = trim_copy(row[object_col]);
      if (!object_name.empty())
        object_order.push_back(object_name);
    }

    if (object_order.empty())
    {
      std::cerr << "[Replay] No object order entries found in: " << csv_path
                << std::endl;
      return false;
    }

    return true;
  }

  bool build_replay_plan(const std::vector<FinalAllocation> &loaded_sequence,
                         const std::vector<std::string> &object_order,
                         AllocationScenarioPlan &plan_out)
  {
    if (object_order.size() != loaded_sequence.size())
    {
      std::cerr << "[Replay] Object count mismatch. CSV has " << object_order.size()
                << " entries but loaded sequence has " << loaded_sequence.size()
                << " tasks." << std::endl;
      return false;
    }

    std::unordered_map<std::string, std::size_t> object_to_index;
    for (std::size_t i = 0; i < loaded_sequence.size(); ++i)
    {
      object_to_index[loaded_sequence[i].object.name] = i;
    }

    plan_out.task_order.clear();
    plan_out.preferred_robot_names_by_original_task.clear();
    plan_out.task_order.reserve(object_order.size());

    std::unordered_set<std::string> seen;
    for (const auto &object_name : object_order)
    {
      if (!seen.insert(object_name).second)
      {
        std::cerr << "[Replay] Duplicate object in replay order: " << object_name
                  << std::endl;
        return false;
      }

      auto it = object_to_index.find(object_name);
      if (it == object_to_index.end())
      {
        std::cerr << "[Replay] Unknown object in replay order: " << object_name
                  << std::endl;
        return false;
      }

      plan_out.task_order.push_back(it->second);
    }

    return true;
  }

  std::string summarize_for_title(const AllocationRunSummary &summary)
  {
    std::ostringstream oss;
    oss << summary.label << " | ";
    if (summary.all_tasks_succeeded && std::isfinite(summary.makespan))
    {
      oss << "feasible, makespan=" << std::fixed << std::setprecision(2)
          << summary.makespan << "s";
    }
    else
    {
      oss << "infeasible, success=" << summary.successful_tasks << "/"
          << (summary.successful_tasks + summary.failed_tasks);
    }
    return oss.str();
  }
} // namespace

int main(int argc, char **argv)
{
  std::string sequence_csv_path = parse_sequence_csv_path(argc, argv);
  int trial_iteration = parse_trial_iteration(argc, argv);
  if (trial_iteration < 0)
    return 1;

  RuntimeOptions runtime_options = parse_runtime_options(argc, argv);
  ReloPush::HandoffInstanceInfo instance_info;
  std::vector<FinalAllocation> loaded_sequence;
  std::unique_ptr<ReloPush::FinalSequenceHandoffServer> handoff_server;
  if (!load_data(runtime_options, instance_info, loaded_sequence, handoff_server))
    return 1;

  print_runtime_options(runtime_options);

  std::vector<std::string> object_order;
  if (!load_object_order_from_csv(sequence_csv_path, trial_iteration, object_order))
    return 1;

  AllocationScenarioPlan replay_plan;
  if (!build_replay_plan(loaded_sequence, object_order, replay_plan))
    return 1;

  std::cout << "[Replay] Source CSV: " << sequence_csv_path << std::endl;
  if (trial_iteration > 0)
  {
    std::cout << "[Replay] Requested trial iteration: " << trial_iteration
              << std::endl;
  }
  std::cout << "[Replay] Order={"
            << preview_task_order(replay_plan, loaded_sequence, loaded_sequence.size())
            << "}" << std::endl;

  AllocationScenarioPlan greedy_plan = make_identity_plan(loaded_sequence.size());
  std::cout << "[Replay] Re-running greedy baseline for side-by-side comparison..."
            << std::endl;
  auto greedy_executed = execute_allocation_scenario(
      loaded_sequence, runtime_options, greedy_plan, "greedy",
      mix_seed(runtime_options.base_random_seed, 0xC0FFEE01u), false);
  print_comparison_line(greedy_executed.summary, greedy_executed.summary.makespan);

  std::cout << "[Replay] Re-running requested sequence..." << std::endl;
  auto replay_executed = execute_allocation_scenario(
      loaded_sequence, runtime_options, replay_plan, "replay-sequence",
      mix_seed(runtime_options.base_random_seed, 0x5245504Cu), true);
  print_comparison_line(replay_executed.summary, greedy_executed.summary.makespan);

  std::string replay_csv_path =
      std::string(CMAKE_SOURCE_DIR) + "/task_execution_log_replay_sequence.csv";
  write_task_csv_log(replay_csv_path, replay_executed.summary.task_rows);

  if (runtime_options.enable_visualization)
  {
    show_results_comparison(
        argc, argv,
        QString::fromStdString(summarize_for_title(greedy_executed.summary)),
        greedy_executed.timetable, greedy_executed.entities, greedy_executed.params,
        QString::fromStdString(summarize_for_title(replay_executed.summary)),
        replay_executed.timetable, replay_executed.entities, replay_executed.params);
  }
  else
  {
    std::cout << "[Replay] Visualization disabled. Pass --visualize to open the "
                 "side-by-side comparison window."
              << std::endl;
  }

  return replay_executed.summary.all_tasks_succeeded ? 0 : 1;
}
