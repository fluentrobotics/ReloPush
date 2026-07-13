#ifndef TRANSITION_LOGGER_H
#define TRANSITION_LOGGER_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

// ==========================================
// Transition logger for the DQN v2 allocation search. See
// MARS/09rl-pretrained-study-plan.md section 3.1.
// ==========================================
//
// Captures every legal candidate at every construction step (not just the
// chosen one) so the corpus supports both offline argmax training and
// per-decision feature analysis.

// phi is the v2 feature vector (DqnV2::kFeatDim entries) for one candidate
// task at one construction step.
struct StepCandidateEntry
{
  std::size_t step = 0;
  std::size_t candidate_task = 0;
  bool chosen = false;
  std::vector<double> phi;
};

// Retrospective outcome of one full rollout, shared across every logged row
// of that rollout (denormalized on purpose -- one CSV row per candidate).
struct RolloutOutcome
{
  bool feasible = false;
  double makespan = -1.0;         // -1 if infeasible
  double return_target = 0.0;     // Monte-Carlo target used for training
  std::size_t first_fail_rank = 0; // == order.size() if feasible
  long first_failed_task = -1;    // -1 if feasible
};

class TransitionLogger
{
public:
  // Opens `path` in append mode; writes the CSV header iff the file is
  // empty or does not yet exist (so multiple runs share one growing corpus).
  explicit TransitionLogger(const std::string &path);

  void log_rollout(const std::string &family, int index, std::uint32_t seed,
                    int iteration,
                    const std::vector<StepCandidateEntry> &step_log,
                    const RolloutOutcome &outcome);

private:
  std::ofstream out_;
};

// Parses "<...>result_seq_<family>_ind<index>.b64" -- family may itself
// contain underscores (e.g. "ReloPush-BOSS_12_objects.txt"). Returns false
// (family_out="unknown", index_out=-1) if the pattern doesn't match.
bool parse_family_index(const std::string &path, std::string &family_out,
                        int &index_out);

#endif // TRANSITION_LOGGER_H
