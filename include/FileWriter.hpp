#ifndef FILEWRITER_HPP
#define FILEWRITER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <variant>

// Include your headers
#include <TaskAllocation.hpp>
#include <State.h>
#include <config.h>


// Function to export data to a text file
void exportToTxt(
    float xMax,
    float yMax,
    const std::vector<ObjectInfo> &objects, // Should have type OBJECT_VERTEX
    const std::vector<GoalInfo> &goals,   // Should have type GOAL_VERTEX
    const Graph &graph,
    const std::string &filename
    )
{
    std::ofstream outfile(filename);
    if (!outfile.is_open())
    {
        std::cerr << "Error: Could not open the file " << filename << " for writing.\n";
        return;
    }

    // Line 1: Boundary
    outfile << xMax << "," << yMax << "\n";

    // Line 2: Objects
    std::string objectsLine;
    for (size_t i = 0; i < objects.size(); ++i)
    {
        const auto &obj = objects[i];
        objectsLine += std::to_string(obj.x) + "," + std::to_string(obj.y) + "," + std::to_string(obj.nominalOrientation);
        if (i != objects.size() - 1)
            objectsLine += ";";
    }
    outfile << objectsLine << "\n";

    // Line 3: Goals
    std::string goalsLine;
    for (size_t i = 0; i < goals.size(); ++i)
    {
        const auto &goal = goals[i];
        goalsLine += std::to_string(goal.x) + "," + std::to_string(goal.y) + "," + std::to_string(goal.nominalOrientation);
        if (i != goals.size() - 1)
            goalsLine += ";";
    }
    outfile << goalsLine << "\n";

    // Lines 4 and onward: Paths from StatePathPtr
    // Iterate through all edges in the graph
    auto edges = boost::edges(graph);
    for (auto it = edges.first; it != edges.second; ++it)
    {
        Edge e = *it;
        const EdgeData &edgeData = graph[e];

        for (const auto &pathVariant : edgeData.paths)
        {
            ReloPush::StatePathPtr statePath;
            // Check if the variant holds a StatePathPtr
            if (std::holds_alternative<ReloPush::StatePathPtr>(pathVariant->path))
            {
                statePath = std::get<ReloPush::StatePathPtr>(pathVariant->path);

            }
            // If needed, handle reloDubinsPath here (currently ignored)
            else if(std::holds_alternative<reloDubinsPath>(pathVariant->path))
            {
                auto dubinsPath = std::get<reloDubinsPath>(pathVariant->path);
                statePath = dubinsPath.interpolate(0.1); // todo: parse map resolution
            }

            if (statePath && !statePath->empty())
            {
                std::string pathLine;
                for (size_t i = 0; i < statePath->size(); ++i)
                {
                    const ReloPush::State &state = (*statePath)[i];
                    pathLine += std::to_string(state.x) + "," + std::to_string(state.y) + "," + std::to_string(state.yaw);
                    if (i != statePath->size() - 1)
                        pathLine += ";";
                }
                outfile << pathLine << "\n";
            }
        }
    }

    outfile.close();
    std::cout << "Data successfully exported to " << filename << "\n";
}

// remove extension from a file name
std::string removeExtension(const std::string& filename) {
    size_t lastDotPos = filename.find_last_of(".");
    if (lastDotPos == std::string::npos) {
        // No dot found, return the original filename
        return filename;
    } else {
        // Return the substring from the beginning to the position of the last dot
        return filename.substr(0, lastDotPos);
    }
}

// Function to write the summary of finalSequence to a file.
// It creates the file if it does not exist, or appends if it does.
void writeFinalSequenceSummary(const std::string &filename_in, int instanceIndex, double planningTime,
                               const std::vector<FinalAllocation>& finalSequence, bool& use_opt)
{
    // change file name to save logs
    auto file_name_wo_ext = removeExtension(filename_in);
    std::string use_opt_str = "_NonOpt";
    if(use_opt)
        use_opt_str = "_Opt";
    std::string filename = file_name_wo_ext + use_opt_str + "_log.txt";

    std::string cmake_log_dir = std::string(CMAKE_SOURCE_DIR) + "/log/";
    // Open file in append mode (creates the file if it does not exist)
    std::ofstream file(cmake_log_dir + filename, std::ios::app);
    if (!file)
    {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // Prepare string streams for each data category.
    std::ostringstream objectNamesStream;
    std::ostringstream goalNamesStream;
    std::ostringstream rowsStream;
    std::ostringstream colsStream;
    std::ostringstream preReloStream;
    std::ostringstream obsReloStream;
    std::ostringstream costsStream;
    double totalCost = 0.0;
    double pushingLength = 0.0;

    // Process each FinalAllocation entry.
    for (const auto &alloc : finalSequence)
    {
        // Object name
        objectNamesStream << alloc.object.name << " ";

        // Goal name
        goalNamesStream << alloc.goal.name << " ";

        // Row and column
        rowsStream << alloc.row << " ";
        colsStream << alloc.col << " ";

        // Pre-relocation flag: if paths is non-empty, we check the first element's preRelo.used flag.
        bool preReloUsed = false;
        if (!alloc.paths.empty())
        {
            preReloUsed = alloc.paths.front().preRelo.used; // adjust if your structure differs
        }
        preReloStream << (preReloUsed ? "true" : "false") << " ";

        // ObsReloPaths flag: true if the pointer is valid and the underlying vector is not empty.
        bool obsReloUsed = (alloc.obsReloPaths && !alloc.obsReloPaths->empty());
        obsReloStream << (obsReloUsed ? "true" : "false") << " ";

        // Cost and accumulate the total cost.
        costsStream << alloc.cost << " ";
        totalCost += alloc.cost;
        pushingLength += alloc.getPushingLength();
    }

    // Write all the collected data to file.
    file << "Filename: " << filename << "\n";
    file << "Instance_Index: " << instanceIndex << "\n";
    file << "Objects: " << objectNamesStream.str() << "\n";
    file << "Goals: " << goalNamesStream.str() << "\n";
    file << "Rows: " << rowsStream.str() << "\n";
    file << "Cols: " << colsStream.str() << "\n";
    file << "PreRelocation_used: " << preReloStream.str() << "\n";
    file << "ObsReloPaths_used: " << obsReloStream.str() << "\n";
    file << "Costs: " << costsStream.str() << "\n";
    file << "Total_Cost: " << totalCost << "\n";
    file << "Pushing_Length(m): " << pushingLength << "\n";
    file << "Planning_Time(ms): " << planningTime << "\n";
    file << "--------------------------" << "\n";

    file.close();
}

#endif // FILEWRITER_HPP
