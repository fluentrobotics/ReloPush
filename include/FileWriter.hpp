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
#include <GraphData.hpp>
#include <State.h>


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



#endif // FILEWRITER_HPP
