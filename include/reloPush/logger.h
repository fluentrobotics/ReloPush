#ifndef LOGGER_H
#define LOGGER_H

#include <vector>
#include <string>

#include "stopwatch.h"

namespace jeeho
{
    class logger
    {
        bool is_success;
        size_t num_relo;
        std::vector<std::string> delivery_sequence;
        float time_graph_plan = 0;
        float time_path_plan = 0;
        float time_assign = 0;
        float time_total = 0;
        float time_reloc_plan = 0;

        logger(bool is_succ, size_t num_relocation, std::vector<std::string>& d_seq, stopWatchSet& time_measurements) 
        : is_success(is_succ), num_relo(num_relocation), delivery_sequence(d_seq)
        {
            // handle stopwatch measurements
            for(auto& it : time_measurements.watches)
            {
                float durationInSeconds = static_cast<float>(it.get_measurement()) / 1000000.0f;
                switch (it.get_type())
                {
                case measurement_type::allPlan :
                    time_total = durationInSeconds;
                    break;
                
                case measurement_type::assign :
                    time_assign += durationInSeconds;
                    break;
                
                case measurement_type::graphPlan :
                    time_graph_plan += durationInSeconds;
                    break;

                case measurement_type::pathPlan :
                    time_path_plan += durationInSeconds;
                    break;

                case measurement_type::relocatePlan :
                    time_reloc_plan += durationInSeconds;
                    break;
                
                default:
                    break;
                }
            }
        }

    };
}

#endif