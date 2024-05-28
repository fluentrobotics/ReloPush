#ifndef LOGGER_H
#define LOGGER_H

#include <vector>
#include <string>
#include <fstream>

#include "stopwatch.h"
#include <config.h>

namespace jeeho
{
    class logger
    {
        private:
            bool is_success;
            size_t num_relo;
            std::vector<std::string> delivery_sequence;
            float time_graph_plan = 0;
            float time_path_plan = 0;
            float time_assign = 0;
            float time_total = 0;
            float time_reloc_plan = 0;

        public:
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

        void write_to_file(std::string file_name)
        {
            std::ofstream outFile(file_name);
            std::string set_delim = "#";
            std::string header_delim = ":";
            std::string elem_delim = ",";

            if (outFile.is_open()) {
                outFile << set_delim << std::endl;
                outFile << "Is Success" << header_delim << (is_success ? "T" : "F") << "\n";
                outFile << "Number of Relocations" << header_delim << num_relo << "\n";

                outFile << "Delivery Sequence" << header_delim;
                for (size_t i = 0; i < delivery_sequence.size(); ++i) {
                    outFile << delivery_sequence[i];
                    if (i < delivery_sequence.size() - 1) {
                        outFile << elem_delim;
                    }
                }
                outFile << "\n";

                outFile << "Time Graph Plan" << header_delim << time_graph_plan << "\n";
                outFile << "Time Path Plan" << header_delim << time_path_plan << "\n";
                outFile << "Time Assign" << header_delim << time_assign << "\n";
                outFile << "Time Total" << header_delim << time_total << "\n";
                outFile << "Time Reloc Plan" << header_delim << time_reloc_plan << "\n";

                outFile.close();
                std::cout << "Data successfully written to " << file_name << std::endl;
            } else {
                std::cerr << "Unable to open file " << file_name << std::endl;
            }
        }

    };
}

#endif