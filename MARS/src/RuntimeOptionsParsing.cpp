#include <RuntimeOptionsParsing.h>
#include <CsvLogging.h>
#include <config.h>
#include <iostream>
#include <sstream>
#include <cctype>
#include <algorithm>
#include <random>
#include <vector>

std::string default_sequence_path()
{
  return relopush_sequence_path("ReloPush-BOSS_12_objects.txt", 10);
}

RuntimeOptions parse_runtime_options(int argc, char **argv)
{
  RuntimeOptions options;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i] ? argv[i] : "";
    if (arg == "--robot-boundary-mode=origin" ||
        arg == "--robot-boundary-origin-only")
    {
      options.robot_boundary_origin_only = true;
    }
    else if (arg == "--robot-boundary-mode=corners")
    {
      options.robot_boundary_origin_only = false;
    }
    else if (arg == "--parking-candidate-mode=expand")
    {
      options.parking_candidate_mode = ParkingCandidateMode::EXPAND;
    }
    else if (arg == "--parking-candidate-mode=connected")
    {
      options.parking_candidate_mode = ParkingCandidateMode::CONNECTED;
    }
    else if (arg == "--parking-candidate-mode=vfh" ||
             arg == "--parking-candidate-mode=connected-vfh")
    {
      options.parking_candidate_mode = ParkingCandidateMode::CONNECTED_VFH;
    }
    else if (arg == "--parking-candidate-mode=reverse-recent" ||
             arg == "--parking-candidate-mode=reverse-path" ||
             arg == "--parking-candidate-mode=retrace")
    {
      options.parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT;
    }
    else if (arg == "--parking-candidate-mode=reverse-recent-shorter" ||
             arg == "--parking-candidate-mode=reverse-recent-refined" ||
             arg == "--parking-candidate-mode=reverse-shorter" ||
             arg == "--parking-candidate-mode=retrace-shorter")
    {
      options.parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT_SHORTER;
    }
    else if (arg == "--parking-candidate-mode=random")
    {
      options.parking_candidate_mode = ParkingCandidateMode::RANDOM;
    }
    else if (arg == "--failed-candidate-idle-parking")
    {
      options.enable_failed_candidate_idle_parking = true;
    }
    else if (arg == "--no-failed-candidate-idle-parking")
    {
      options.enable_failed_candidate_idle_parking = false;
    }
    else if (arg.rfind("--failed-candidate-initial-transit-threshold=", 0) == 0)
    {
      std::string value = arg.substr(
          std::string("--failed-candidate-initial-transit-threshold=").size());
      try
      {
        options.failed_candidate_initial_transit_failure_threshold =
            std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --failed-candidate-initial-transit-threshold value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg == "--enable-order-learning" ||
             arg == "--order-learning")
    {
      options.enable_order_constraint_learning = true;
    }
    else if (arg == "--disable-order-learning" ||
             arg == "--no-order-learning")
    {
      options.enable_order_constraint_learning = false;
    }
    else if (arg == "--early-abort")
    {
      options.early_abort_eval_on_failure = true;
    }
    else if (arg == "--no-early-abort")
    {
      options.early_abort_eval_on_failure = false;
    }
    else if (arg == "--search-mode=lns" || arg == "--lns")
    {
      options.search_improvement_mode = SearchImprovementMode::LNS;
    }
    else if (arg == "--search-mode=dqn" || arg == "--dqn")
    {
      options.search_improvement_mode = SearchImprovementMode::DQN;
    }
    else if (arg.rfind("--dqn-learning-rate=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-learning-rate=").size());
      try
      {
        options.dqn_learning_rate = std::max(1e-6, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-learning-rate value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-epsilon-start=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-epsilon-start=").size());
      try
      {
        options.dqn_epsilon_start =
            std::min(1.0, std::max(0.0, std::stod(value)));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-epsilon-start value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-epsilon-end=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-epsilon-end=").size());
      try
      {
        options.dqn_epsilon_end = std::min(1.0, std::max(0.0, std::stod(value)));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-epsilon-end value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-grad-steps=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-grad-steps=").size());
      try
      {
        options.dqn_grad_steps_per_iter = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-grad-steps value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-minibatch=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-minibatch=").size());
      try
      {
        options.dqn_minibatch_size = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-minibatch value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-hidden=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-hidden=").size());
      try
      {
        options.dqn_hidden_units = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-hidden value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-features=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-features=").size());
      try
      {
        int parsed = std::stoi(value);
        options.dqn_feature_version = std::max(1, std::min(4, parsed));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-features value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-explore-ref-bias=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-explore-ref-bias=").size());
      try
      {
        options.dqn_explore_ref_bias =
            std::min(1.0, std::max(0.0, std::stod(value)));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-explore-ref-bias value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-learned-hard-evidence=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-learned-hard-evidence=").size());
      try
      {
        options.dqn_learned_hard_evidence = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-learned-hard-evidence value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-log-transitions=", 0) == 0)
    {
      options.dqn_log_transitions_path =
          arg.substr(std::string("--dqn-log-transitions=").size());
    }
    else if (arg.rfind("--dqn-scoring=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-scoring=").size());
      if (value == "penalty")
      {
        options.dqn_scoring_mode = 0;
      }
      else if (value == "decomposed")
      {
        options.dqn_scoring_mode = 1;
      }
      else
      {
        std::cerr << "[Warn] Invalid --dqn-scoring value: '" << value
                  << "'. Keeping default (penalty)." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-fail-threshold=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-fail-threshold=").size());
      try
      {
        options.dqn_fail_threshold = std::min(1.0, std::max(0.0, std::stod(value)));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-fail-threshold value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-fail-pos-weight=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-fail-pos-weight=").size());
      try
      {
        options.dqn_fail_pos_weight = std::max(0.0, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-fail-pos-weight value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg == "--dqn-relabel-executed")
    {
      options.dqn_relabel_executed = true;
    }
    else if (arg.rfind("--dqn-relabel-executed=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-relabel-executed=").size());
      if (value == "1" || value == "true")
        options.dqn_relabel_executed = true;
      else if (value == "0" || value == "false")
        options.dqn_relabel_executed = false;
      else
        std::cerr << "[Warn] Invalid --dqn-relabel-executed value: '" << value
                  << "'. Keeping default (off)." << std::endl;
    }
    else if (arg.rfind("--dqn-init-weights=", 0) == 0)
    {
      options.dqn_init_weights_path =
          arg.substr(std::string("--dqn-init-weights=").size());
    }
    else if (arg.rfind("--dqn-init-fail-weights=", 0) == 0)
    {
      options.dqn_init_fail_weights_path =
          arg.substr(std::string("--dqn-init-fail-weights=").size());
    }
    else if (arg.rfind("--dqn-finetune-lr=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-finetune-lr=").size());
      try
      {
        options.dqn_finetune_learning_rate = std::max(1e-6, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-finetune-lr value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-finetune-epsilon-start=", 0) == 0)
    {
      std::string value =
          arg.substr(std::string("--dqn-finetune-epsilon-start=").size());
      try
      {
        options.dqn_finetune_epsilon_start =
            std::min(1.0, std::max(0.0, std::stod(value)));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-finetune-epsilon-start value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-finetune-epsilon-end=", 0) == 0)
    {
      std::string value =
          arg.substr(std::string("--dqn-finetune-epsilon-end=").size());
      try
      {
        options.dqn_finetune_epsilon_end =
            std::min(1.0, std::max(0.0, std::stod(value)));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-finetune-epsilon-end value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg == "--dqn-freeze-model")
    {
      options.dqn_freeze_model = true;
    }
    else if (arg.rfind("--export-geometry=", 0) == 0)
    {
      options.export_geometry_path =
          arg.substr(std::string("--export-geometry=").size());
    }
    else if (arg.rfind("--geometry-k=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--geometry-k=").size());
      try
      {
        options.geometry_export_k = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --geometry-k value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--export-pg-tables=", 0) == 0)
    {
      options.export_pg_tables_path =
          arg.substr(std::string("--export-pg-tables=").size());
    }
    else if (arg.rfind("--eval-plans=", 0) == 0)
    {
      options.eval_plans_path =
          arg.substr(std::string("--eval-plans=").size());
    }
    else if (arg.rfind("--eval-plans-out=", 0) == 0)
    {
      options.eval_plans_out_path =
          arg.substr(std::string("--eval-plans-out=").size());
    }
    else if (arg.rfind("--eval-plans-timing-out=", 0) == 0)
    {
      options.eval_plans_timing_out_path =
          arg.substr(std::string("--eval-plans-timing-out=").size());
    }
    else if (arg.rfind("--eval-plans-result-out-dir=", 0) == 0)
    {
      // NOTE: checked before "--eval-plans-result-out=" below since that
      // prefix is a strict prefix of this one.
      options.eval_plans_result_out_dir =
          arg.substr(std::string("--eval-plans-result-out-dir=").size());
    }
    else if (arg.rfind("--eval-plans-result-out=", 0) == 0)
    {
      options.eval_plans_result_out_path =
          arg.substr(std::string("--eval-plans-result-out=").size());
    }
    else if (arg.rfind("--play-result=", 0) == 0)
    {
      options.play_result_path =
          arg.substr(std::string("--play-result=").size());
    }
    else if (arg.rfind("--export-decision-time-log=", 0) == 0)
    {
      options.export_decision_time_log_path =
          arg.substr(std::string("--export-decision-time-log=").size());
    }
    else if (arg.rfind("--random-seed=", 0) == 0)
    {
      std::string seed_text = arg.substr(std::string("--random-seed=").size());
      try
      {
        unsigned long parsed = std::stoul(seed_text);
        options.base_random_seed = static_cast<std::uint32_t>(parsed);
        options.has_fixed_random_seed = true;
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --random-seed value: '" << seed_text
                  << "'. Falling back to non-deterministic seed." << std::endl;
      }
    }
    else if (arg.rfind("--lns-threads=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--lns-threads=").size());
      try
      {
        options.lns_threads = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --lns-threads value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--num-robots=", 0) == 0 ||
             arg.rfind("--robot-count=", 0) == 0)
    {
      const std::string num_prefix = "--num-robots=";
      const std::string count_prefix = "--robot-count=";
      std::string value = arg.rfind(num_prefix, 0) == 0
                              ? arg.substr(num_prefix.size())
                              : arg.substr(count_prefix.size());
      try
      {
        options.robot_count = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid robot count value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--planner-expansion-threads=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--planner-expansion-threads=").size());
      try
      {
        options.planner_expansion_threads = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --planner-expansion-threads value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--max-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--max-search-iters=").size());
      try
      {
        options.max_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --max-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--safe-parking-max-search-iters=", 0) == 0)
    {
      std::string value =
          arg.substr(std::string("--safe-parking-max-search-iters=").size());
      try
      {
        options.safe_parking_max_search_iterations =
            std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --safe-parking-max-search-iters value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--safe-parking-expand-iters=", 0) == 0)
    {
      // Bounds the safe-parking connected-candidate frontier sweeps (see
      // Params::safe_parking_expand_max_iterations); distinct from
      // --safe-parking-max-search-iters= above, which only bounds the
      // PHAStar relocation search's own iteration cap.
      std::string value =
          arg.substr(std::string("--safe-parking-expand-iters=").size());
      try
      {
        options.default_safe_parking_expand_iterations =
            std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --safe-parking-expand-iters value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--fine-segment-max-search-iters=", 0) == 0)
    {
      std::string value =
          arg.substr(std::string("--fine-segment-max-search-iters=").size());
      try
      {
        options.fine_segment_max_search_iterations =
            std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --fine-segment-max-search-iters value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--contact-boundary-max-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(
          std::string("--contact-boundary-max-search-iters=").size());
      try
      {
        options.contact_boundary_max_search_iterations =
            std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --contact-boundary-max-search-iters value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg == "--tier-triage")
    {
      options.enable_tier_triage = true;
    }
    else if (arg == "--no-tier-triage")
    {
      options.enable_tier_triage = false;
    }
    else if (arg == "--tier-gate")
    {
      options.enable_tier_gate = true;
    }
    else if (arg == "--no-tier-gate")
    {
      options.enable_tier_gate = false;
    }
    else if (arg.rfind("--collision-check-time-step=", 0) == 0)
    {
      std::string value =
          arg.substr(std::string("--collision-check-time-step=").size());
      try
      {
        options.default_collision_check_time_step =
            std::max(1e-6, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --collision-check-time-step value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--fine-collision-check-time-step=", 0) == 0)
    {
      std::string value =
          arg.substr(std::string("--fine-collision-check-time-step=").size());
      try
      {
        options.fine_segment_collision_check_time_step =
            std::max(1e-6, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --fine-collision-check-time-step value: '"
                  << value << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--retraction-distance=", 0) == 0 ||
             arg.rfind("--retract-distance=", 0) == 0)
    {
      const std::string full_prefix = "--retraction-distance=";
      const std::string short_prefix = "--retract-distance=";
      std::string value = arg.rfind(full_prefix, 0) == 0
                              ? arg.substr(full_prefix.size())
                              : arg.substr(short_prefix.size());
      try
      {
        options.retraction_distance = std::max(0.0, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --retraction-distance value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--assignment-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--assignment-search-iters=").size());
      try
      {
        options.assignment_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --assignment-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--sequence-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--sequence-search-iters=").size());
      try
      {
        int parsed_iters = std::max(0, std::stoi(value));
        options.local_sequence_search_iterations = parsed_iters;
        options.shuffle_sequence_search_iterations = parsed_iters;
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --sequence-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--local-sequence-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--local-sequence-search-iters=").size());
      try
      {
        options.local_sequence_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --local-sequence-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--shuffle-sequence-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--shuffle-sequence-search-iters=").size());
      try
      {
        options.shuffle_sequence_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --shuffle-sequence-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--lns-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--lns-iters=").size());
      try
      {
        options.lns_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --lns-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--dqn-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--dqn-iters=").size());
      try
      {
        options.dqn_iterations = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --dqn-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg == "--lns-fine-segment-retry")
    {
      options.enable_lns_fine_segment_retry = true;
    }
    else if (arg == "--no-lns-fine-segment-retry")
    {
      options.enable_lns_fine_segment_retry = false;
    }
    else if (arg == "--lns-reassign-only" ||
             arg == "--lns-preserve-task-sequence")
    {
      options.lns_reassign_only = true;
    }
    else if (arg == "--lns-task-reassign" ||
             arg == "--no-lns-reassign-only" ||
             arg == "--lns-allow-sequence-edits")
    {
      options.lns_reassign_only = false;
    }
    else if (arg == "--visualize" || arg == "--visualization")
    {
      options.enable_visualization = true;
    }
    else if (arg == "--no-visualization")
    {
      options.enable_visualization = false;
    }
    else if (arg == "--visualize-relopush-plan" ||
             arg == "--debug-relopush-plan")
    {
      options.visualize_relopush_plan = true;
    }
    else if (arg == "--no-visualize-relopush-plan")
    {
      options.visualize_relopush_plan = false;
    }
    else if (arg == "--result-summary-figure" ||
             arg == "--visualize-result-summary" ||
             arg == "--visualize-result-traces")
    {
      options.enable_result_summary_figure = true;
    }
    else if (arg == "--no-result-summary-figure")
    {
      options.enable_result_summary_figure = false;
    }
    else if (arg.rfind("--result-summary-output=", 0) == 0)
    {
      options.result_summary_output_path =
          arg.substr(std::string("--result-summary-output=").size());
      options.enable_result_summary_figure = true;
    }
    else if (arg.rfind("--result-summary-gap=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--result-summary-gap=").size());
      try
      {
        options.result_summary_subplot_gap = std::max(0.0, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --result-summary-gap value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--fixed-order=", 0) == 0)
    {
      options.fixed_order_path_or_list =
          arg.substr(std::string("--fixed-order=").size());
    }
    else if (arg.rfind("--robot-trace-colors=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--robot-trace-colors=").size());
      std::vector<std::string> parsed_colors;
      std::stringstream ss(value);
      std::string color;
      while (std::getline(ss, color, ','))
      {
        color.erase(std::remove_if(color.begin(), color.end(),
                                   [](unsigned char ch)
                                   { return std::isspace(ch); }),
                    color.end());
        if (!color.empty())
        {
          parsed_colors.push_back(color);
        }
      }
      if (!parsed_colors.empty())
      {
        options.robot_trace_colors = parsed_colors;
      }
    }
    else if (arg == "--integrated-mode" ||
             arg == "--integrated")
    {
      options.integrated_mode = true;
    }
    else if (arg == "--run-on-robots" ||
             arg == "--run-on-robot")
    {
      options.run_on_robots = true;
    }
    else if (arg.rfind("--robot-port-start=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--robot-port-start=").size());
      try
      {
        options.robot_controller_port_start = std::stoi(value);
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --robot-port-start value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--mpc-vesc-port-start=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--mpc-vesc-port-start=").size());
      try
      {
        options.mpc_vesc_port_start = std::stoi(value);
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --mpc-vesc-port-start value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--mpc-localization-port-start=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--mpc-localization-port-start=").size());
      try
      {
        options.mpc_localization_port_start = std::stoi(value);
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --mpc-localization-port-start value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--mpc-vesc-ip=", 0) == 0)
    {
      options.mpc_vesc_ip = arg.substr(std::string("--mpc-vesc-ip=").size());
    }
    else if (arg.rfind("--mpc-localization-ip=", 0) == 0)
    {
      options.mpc_localization_ip = arg.substr(std::string("--mpc-localization-ip=").size());
    }
    else if (arg.rfind("--handoff-endpoint=", 0) == 0)
    {
      options.handoff_endpoint = arg.substr(std::string("--handoff-endpoint=").size());
    }
    else if (arg.rfind("--mars-endpoint=", 0) == 0)
    {
      options.handoff_endpoint = arg.substr(std::string("--mars-endpoint=").size());
    }
    else if (arg.rfind("--sequence-file=", 0) == 0)
    {
      options.input_sequence_path = arg.substr(std::string("--sequence-file=").size());
    }
    else if (arg.rfind("--input-sequence=", 0) == 0)
    {
      options.input_sequence_path = arg.substr(std::string("--input-sequence=").size());
    }
    else if (arg == "--debug-vis")
    {
      options.debug_vis = true;
    }
    else if (arg == "--no-debug-vis")
    {
      options.debug_vis = false;
    }
  }

  if (!options.has_fixed_random_seed)
  {
    std::random_device rd;
    options.base_random_seed = rd();
  }

  // Decomposed scoring (Decision 2) needs the v3/v4 (task, robot) explicit
  // feature machinery; downgrade rather than abort if the flags conflict, so
  // a single bad combination doesn't kill an otherwise-valid batch run.
  if (options.dqn_scoring_mode == 1 && options.dqn_feature_version < 3)
  {
    std::cerr << "[Warn] --dqn-scoring=decomposed requires --dqn-features=3 or 4 (got "
              << options.dqn_feature_version << "). Falling back to --dqn-scoring=penalty."
              << std::endl;
    options.dqn_scoring_mode = 0;
  }

  return options;
}
