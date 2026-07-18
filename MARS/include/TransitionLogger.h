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
  // v3 only: robot index this candidate pairs with `candidate_task`. v1/v2
  // never set this field, so it stays at the default (-1); kept additive so
  // the v1/v2 CSV logging path (TransitionLogger) is untouched.
  long robot = -1;
  bool chosen = false;
  std::vector<double> phi;
  // Executed-robot relabeled corpus logging only (options.dqn_relabel_executed,
  // see ingest_rollout_v3_relabeled in DqnAllocationSearch.cpp): meaningful
  // ONLY on the chosen row of a step -- the actually-executed robot index
  // (-1 if no definite executed robot: failing step under early abort,
  // unexecuted tail, or an unresolved robot name) and whether it differs from
  // the intended (chosen) robot. Left at these defaults, and unused, by every
  // other logging path (v1/v2, and v3/v4 without relabeling).
  long executed_robot = -1;
  bool diverged = false;
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
  // extended_v3_schema selects the header/row format written by log_rollout:
  // false (default) is the original v1/v2 9-phi-column schema, byte-identical
  // to before this parameter existed. true is the v3/v4 12-phi-column schema
  // with candidate_robot/executed_robot/diverged columns, used only for
  // options.dqn_relabel_executed corpus logging (see run_dqn_search in
  // DqnAllocationSearch.cpp).
  explicit TransitionLogger(const std::string &path, bool extended_v3_schema = false);

  void log_rollout(const std::string &family, int index, std::uint32_t seed,
                    int iteration,
                    const std::vector<StepCandidateEntry> &step_log,
                    const RolloutOutcome &outcome);

private:
  std::ofstream out_;
  bool extended_v3_schema_ = false;
};

// Parses "<...>result_seq_<family>_ind<index>.b64" -- family may itself
// contain underscores (e.g. "ReloPush-BOSS_12_objects.txt"). Returns false
// (family_out="unknown", index_out=-1) if the pattern doesn't match.
bool parse_family_index(const std::string &path, std::string &family_out,
                        int &index_out);

#endif // TRANSITION_LOGGER_H
