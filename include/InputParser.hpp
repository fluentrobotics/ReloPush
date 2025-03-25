#ifndef INPUTPARSER_HPP
#define INPUTPARSER_HPP

#include <string>
#include <vector>
#include "GraphBuilder.hpp"  // or wherever ObjectInfo & GoalInfo are defined
#include <config.h> // for parsing CMake Source Path

/**
 * @brief Parse an input file that contains:
 *  1) A workspace boundary: xMin xMax yMin yMax
 *  2) # of objects, then each object line: name x y nomOri sides radius
 *  3) # of goals, then each goal line: name x y nomOri sides radius
 *
 * @param filename       The file path
 * @param boundary       Output: workspace boundary
 * @param objects        Output: object data
 * @param goals          Output: goal data
 * @return true if parsed successfully, false otherwise
 */
bool parseInputFile(
    const std::string &filename,
    WorkspaceBoundary &boundary,
    std::unordered_map<std::string, ObjectInfo> &objects,
    std::unordered_map<std::string, GoalInfo> &goals,
    std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs);

// ---------------------------------------------------------------------------
// Helper Function 1: Parse and Initialize
// ---------------------------------------------------------------------------
bool parseAndInitialize(const std::string &filename,
                        WorkspaceBoundary &boundary,
                        std::unordered_map<std::string, ObjectInfo> &objects,
                        std::unordered_map<std::string, GoalInfo> &goals,
                        std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs);

#endif // INPUTPARSER_HPP
