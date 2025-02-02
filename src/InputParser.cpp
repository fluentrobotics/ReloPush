#include "InputParser.hpp"
#include <fstream>
#include <iostream>

bool parseInputFile(
    const std::string &filename,
    WorkspaceBoundary &boundary,
    std::unordered_map<std::string, ObjectInfo> &objects,
    std::unordered_map<std::string, GoalInfo> &goals,
    std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs)
{
    std::ifstream fin(std::string(CMAKE_SOURCE_DIR) + "/input/" + filename);
    if (!fin.is_open())
    {
        std::cerr << "Error opening file: " << filename << "\n";
        return false;
    }

    //
    // 1) Read workspace boundary (optional)
    //
    if (!(fin >> boundary.xMin >> boundary.xMax >> boundary.yMin >> boundary.yMax))
    {
        std::cerr << "Failed to parse workspace boundary.\n";
        return false;
    }

    //
    // 2) Number of objects
    //
    int numObjects = 0;
    if (!(fin >> numObjects) || numObjects < 0)
    {
        std::cerr << "Invalid or missing number of objects.\n";
        return false;
    }

    //
    // 3) Read each object
    //
    objects.clear();
    for (int i = 0; i < numObjects; ++i)
    {
        ObjectInfo oi;
        // e.g.: name  x  y  nominalOrientation  numberOfSides  enclosingRadius
        if (!(fin >> oi.name >> oi.x >> oi.y
              >> oi.nominalOrientation
              >> oi.numberOfSides
              >> oi.enclosingRadius))
        {
            std::cerr << "Failed to parse object info line " << (i + 1) << "\n";
            return false;
        }
        objects[oi.name] = oi; // Insert into unordered_map
    }

    //
    // 4) Number of goals
    //
    int numGoals = 0;
    if (!(fin >> numGoals) || numGoals < 0)
    {
        std::cerr << "Invalid or missing number of goals.\n";
        return false;
    }

    //
    // 5) Read each goal
    //
    goals.clear();
    for (int i = 0; i < numGoals; ++i)
    {
        GoalInfo gi;
        // e.g.: name  x  y  nominalOrientation  numberOfSides  enclosingRadius
        if (!(fin >> gi.name >> gi.x >> gi.y
              >> gi.nominalOrientation
              >> gi.numberOfSides
              >> gi.enclosingRadius))
        {
            std::cerr << "Failed to parse goal info line " << (i + 1) << "\n";
            return false;
        }
        goals[gi.name] = gi; // Insert into unordered_map
    }

    //
    // 6) Number of object-goal pairs
    //
    int numPairs = 0;
    if (!(fin >> numPairs) || numPairs < 0)
    {
        std::cerr << "Invalid or missing number of object-goal pairs.\n";
        return false;
    }

    //
    // 7) Read each object-goal pair
    //
    objGoalPairs.clear();
    //objGoalPairs.reserve(numPairs);
    for (int i = 0; i < numPairs; ++i)
    {
        ObjectGoalPair og;
        if (!(fin >> og.objectName >> og.goalName))
        {
            std::cerr << "Failed to parse object-goal pair line " << (i + 1) << "\n";
            return false;
        }
        //objGoalPairs.push_back(og);
        objGoalPairs[og.objectName] = og;
    }

    fin.close();
    return true;
}

/*
bool parseInputFile(
    const std::string &filename,
    WorkspaceBoundary &boundary,
    std::vector<ObjectInfo> &objects,
    std::vector<GoalInfo> &goals,
    std::vector<ObjectGoalPair> &objGoalPairs)
{
    std::ifstream fin(std::string(CMAKE_SOURCE_DIR) + "/input/" + filename);
    if (!fin.is_open())
    {
        std::cerr << "Error opening file: " << filename << "\n";
        return false;
    }

    //
    // 1) Read workspace boundary (optional)
    //
    if (!(fin >> boundary.xMin >> boundary.xMax >> boundary.yMin >> boundary.yMax))
    {
        std::cerr << "Failed to parse workspace boundary.\n";
        return false;
    }

    //
    // 2) Number of objects
    //
    int numObjects = 0;
    if (!(fin >> numObjects) || numObjects < 0)
    {
        std::cerr << "Invalid or missing number of objects.\n";
        return false;
    }

    //
    // 3) Read each object
    //
    objects.clear();
    objects.reserve(numObjects);
    for (int i = 0; i < numObjects; ++i)
    {
        ObjectInfo oi;
        // e.g.: name  x  y  nominalOrientation  numberOfSides  enclosingRadius
        if (!(fin >> oi.name >> oi.x >> oi.y
              >> oi.nominalOrientation
              >> oi.numberOfSides
              >> oi.enclosingRadius))
        {
            std::cerr << "Failed to parse object info line " << (i + 1) << "\n";
            return false;
        }
        objects.push_back(oi);
    }

    //
    // 4) Number of goals
    //
    int numGoals = 0;
    if (!(fin >> numGoals) || numGoals < 0)
    {
        std::cerr << "Invalid or missing number of goals.\n";
        return false;
    }

    //
    // 5) Read each goal
    //
    goals.clear();
    goals.reserve(numGoals);
    for (int i = 0; i < numGoals; ++i)
    {
        GoalInfo gi;
        // e.g.: name  x  y  nominalOrientation  numberOfSides  enclosingRadius
        if (!(fin >> gi.name >> gi.x >> gi.y
              >> gi.nominalOrientation
              >> gi.numberOfSides
              >> gi.enclosingRadius))
        {
            std::cerr << "Failed to parse goal info line " << (i + 1) << "\n";
            return false;
        }
        goals.push_back(gi);
    }

    //
    // 6) Number of object-goal pairs
    //
    int numPairs = 0;
    if (!(fin >> numPairs) || numPairs < 0)
    {
        std::cerr << "Invalid or missing number of object-goal pairs.\n";
        return false;
    }

    //
    // 7) Read each object-goal pair
    //
    objGoalPairs.clear();
    objGoalPairs.reserve(numPairs);
    for (int i = 0; i < numPairs; ++i)
    {
        ObjectGoalPair og;
        if (!(fin >> og.objectName >> og.goalName))
        {
            std::cerr << "Failed to parse object-goal pair line " << (i + 1) << "\n";
            return false;
        }
        objGoalPairs.push_back(og);
    }

    fin.close();
    return true;
}
*/
// ---------------------------------------------------------------------------
// Helper Function 1: Parse and Initialize
// ---------------------------------------------------------------------------
bool parseAndInitialize(const std::string &filename,
                        WorkspaceBoundary &boundary,
                        std::unordered_map<std::string, ObjectInfo> &objects,
                        std::unordered_map<std::string, GoalInfo> &goals,
                        std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs)
{
    if (!parseInputFile(filename, boundary, objects, goals, objGoalPairs))
    {
        std::cerr << "Parse failed.\n";
        return false;
    }
    return true;
}
