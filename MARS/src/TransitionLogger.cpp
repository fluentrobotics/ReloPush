#include <TransitionLogger.h>

#include <filesystem>
#include <iomanip>
#include <regex>

namespace
{

bool file_is_empty_or_missing(const std::string &path)
{
  std::error_code ec;
  if (!std::filesystem::exists(path, ec))
    return true;
  return std::filesystem::file_size(path, ec) == 0;
}

} // namespace

TransitionLogger::TransitionLogger(const std::string &path, bool extended_v3_schema)
    : extended_v3_schema_(extended_v3_schema)
{
  const bool write_header = file_is_empty_or_missing(path);

  const std::filesystem::path fs_path(path);
  const std::filesystem::path parent = fs_path.parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);

  out_.open(path, std::ios::app);
  if (write_header && out_.is_open())
  {
    if (extended_v3_schema_)
    {
      out_ << "family,index,seed,iteration,step,candidate_task,candidate_robot,chosen,"
              "phi0,phi1,phi2,phi3,phi4,phi5,phi6,phi7,phi8,phi9,phi10,phi11,"
              "feasible,makespan,return_target,first_fail_rank,first_failed_task,"
              "executed_robot,diverged\n";
    }
    else
    {
      out_ << "family,index,seed,iteration,step,candidate_task,chosen,"
              "phi0,phi1,phi2,phi3,phi4,phi5,phi6,phi7,phi8,"
              "feasible,makespan,return_target,first_fail_rank,first_failed_task\n";
    }
  }
}

void TransitionLogger::log_rollout(const std::string &family, int index,
                                   std::uint32_t seed, int iteration,
                                   const std::vector<StepCandidateEntry> &step_log,
                                   const RolloutOutcome &outcome)
{
  if (!out_.is_open())
    return;

  out_ << std::setprecision(9);
  for (const auto &entry : step_log)
  {
    if (extended_v3_schema_)
    {
      out_ << family << "," << index << "," << seed << "," << iteration << ","
          << entry.step << "," << entry.candidate_task << "," << entry.robot << ","
          << (entry.chosen ? 1 : 0);
      for (std::size_t i = 0; i < 12; ++i)
      {
        out_ << ",";
        if (i < entry.phi.size())
          out_ << entry.phi[i];
        // v3's 10-dim phi leaves phi10/phi11 as empty fields (not 0.0) --
        // distinguishes "not applicable to this feature version" from an
        // actual zero-valued v4 feature.
      }
      out_ << "," << (outcome.feasible ? 1 : 0) << "," << outcome.makespan << ","
          << outcome.return_target << "," << outcome.first_fail_rank << ","
          << outcome.first_failed_task << ",";
      if (entry.chosen)
        out_ << entry.executed_robot;
      out_ << ",";
      if (entry.chosen)
        out_ << (entry.diverged ? 1 : 0);
      out_ << "\n";
    }
    else
    {
      out_ << family << "," << index << "," << seed << "," << iteration << ","
          << entry.step << "," << entry.candidate_task << ","
          << (entry.chosen ? 1 : 0);
      for (std::size_t i = 0; i < 9; ++i)
        out_ << "," << (i < entry.phi.size() ? entry.phi[i] : 0.0);
      out_ << "," << (outcome.feasible ? 1 : 0) << "," << outcome.makespan << ","
          << outcome.return_target << "," << outcome.first_fail_rank << ","
          << outcome.first_failed_task << "\n";
    }
  }
}

bool parse_family_index(const std::string &path, std::string &family_out,
                        int &index_out)
{
  static const std::regex pattern(R"(result_seq_(.+)_ind(\d+)\.b64$)");
  std::smatch match;
  if (std::regex_search(path, match, pattern))
  {
    family_out = match[1].str();
    index_out = std::stoi(match[2].str());
    return true;
  }
  family_out = "unknown";
  index_out = -1;
  return false;
}
