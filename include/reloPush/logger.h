#ifndef LOGGER_H
#define LOGGER_H

#include <vector>
#include <string>
#include <fstream>

#include "stopwatch.h"
#include <config.h>
#include <reloPush/data_collector.h> 

namespace jeeho
{
    class logger
    {
        private:
            bool is_success;
            size_t num_relo;
            size_t num_pre_relo;
            size_t num_temp_relo;
            std::vector<std::string> delivery_sequence;
            float time_graph_plan = 0;
            float time_graph_const = 0; //graph generation
            float time_path_plan = 0;
            float time_assign = 0;
            float time_total = 0;
            float time_reloc_plan = 0;
            std::string testdata_name;
            std::string mode_name;
            int testdata_ind;
            DataCollector dataColtr;
            float push_length = 0;
            float approach_length = 0; // path length for approaching objects

        public:
        /*
            logger(bool is_succ, size_t num_relocation, std::vector<std::string>& d_seq, stopWatchSet& time_measurements, std::string data_name, int data_ind, std::string mode) 
            : is_success(is_succ), num_relo(num_relocation), delivery_sequence(d_seq), testdata_name(data_name), testdata_ind(data_ind), mode_name(mode)
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

                    case measurement_type::graphConst :
                        time_graph_const += durationInSeconds;
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
        */
            logger(bool is_succ, DataCollector dCol, std::string data_name, int data_ind, std::string mode) 
            : is_success(is_succ), dataColtr(dCol), testdata_name(data_name), testdata_ind(data_ind), mode_name(mode)
            {
                // handle stopwatch measurements
                for(auto& it : dataColtr.stopWatches.watches)
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

                    case measurement_type::graphConst :
                        time_graph_const += durationInSeconds;
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

                // push lengths
                for(auto& it : dataColtr.pathInfoList.paths)
                {
                    switch(it.type)
                    {
                        case moveType::pre:
                            push_length += it.pathLength();
                            break;
                        
                        case moveType::temp:
                            push_length += it.pathLength();
                            break;

                        case moveType::final:
                            push_length += it.pathLength();
                            break;

                        case moveType::app:
                            approach_length += it.pathLength();
                            break;
                    }
                }

                // num relocations
                num_pre_relo = dataColtr.pathInfoList.count_pre_relocations();
                num_temp_relo = dataColtr.pathInfoList.count_temp_relocations();
                num_relo = num_pre_relo + num_temp_relo;
            }

            void write_to_file(std::string file_name)
            {
                std::ofstream outFile(file_name,std::ios_base::app);
                std::string set_delim = "#";
                std::string header_delim = ":";
                std::string elem_delim = ",";

                if (outFile.is_open()) {
                    outFile << set_delim << std::endl;
                    outFile << "data_name" << header_delim << testdata_name << std::endl;
                    outFile << "mode" << header_delim << mode_name << std::endl;
                    outFile << "index" << header_delim << testdata_ind << std::endl;
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
                    outFile << "Time Graph Const" << header_delim << time_graph_const << "\n";
                    outFile << "Time Graph Plan" << header_delim << time_graph_plan << "\n";
                    outFile << "Time Assign" << header_delim << time_assign << "\n";
                    
                    outFile << "Time Reloc Plan" << header_delim << time_reloc_plan << "\n";

                    // task time
                    outFile << "Time Task" << header_delim << time_graph_const + time_graph_plan + time_reloc_plan << "\n";
                    // motion planning time
                    outFile << "Time MP" << header_delim << time_path_plan << "\n";
                    // total time
                    outFile << "Time Total" << header_delim << time_total << "\n";

                    // num pre reloc
                    outFile << "Num Pre-relo" << header_delim << num_pre_relo << "\n";

                    // num temp reloc
                    outFile << "Num Temp-relo" << header_delim << num_temp_relo<< "\n";

                    // num total reloc
                    outFile << "Num Total relo" << header_delim << num_relo<< "\n";
                    
                    // push length
                    outFile << "Length Pushing" << header_delim << push_length << "\n";

                    // approach length
                    outFile << "Length Approaching" << header_delim << approach_length << "\n";

                    // total length
                    outFile << "Length Approaching" << header_delim << push_length + approach_length << "\n";


                    outFile.close();
                    std::cout << "Data successfully written to " << file_name << std::endl;
                } else {
                    std::cerr << "Unable to open file " << file_name << std::endl;
                }
            }

    };
}

#endif