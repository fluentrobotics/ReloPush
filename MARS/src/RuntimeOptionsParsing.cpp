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
  return relopush_sequence_path("ReloPush-BOSS_10_objects.txt", 80);
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
    else if (arg == "--no-lns-reassign-only" ||
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

  return options;
}
