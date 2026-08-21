#include <ReloPush/batchInstanceParcer.hpp>
#include <ReloPush/ObjectInfo.hpp>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

// Unit test for the optional `ws:<x_max>,<y_max>` instance-file section
// added to parse_instance_from_file() (src/ReloPush/BatchInstanceParcer.cpp).
//
// Verifies:
//  - a line with a `ws:` section populates has_ws/ws_x/ws_y correctly,
//    regardless of where the `ws:` section sits among the other `!`-separated
//    sections;
//  - a line without a `ws:` section leaves has_ws == false and never
//    touches ws_x/ws_y;
//  - the existing 6-arg call signature (no ws out-params) still compiles and
//    runs, matching how test/test_parse_instance.cpp calls it.

namespace
{
    void write_fixture_file(const std::string &abs_path)
    {
        std::ofstream f(abs_path);
        if (!f)
        {
            throw std::runtime_error("Unable to create test fixture file: " + abs_path);
        }
        // line 0: ws section first, before mo/robot/goal/assign
        f << "ws:4.5,4.5!mo:b1,0.5,0.5,0.0,4!robot:0.1,0.1,0.0!goal:d1,1.0,1.0,0.0,4!assign:b1,d1\n";
        // line 1: ws section last, after assign
        f << "mo:b1,0.5,0.5,0.0,4!robot:0.1,0.1,0.0!goal:d1,1.0,1.0,0.0,4!assign:b1,d1!ws:2.25,2.25\n";
        // line 2: no ws section at all (legacy format)
        f << "mo:b1,0.5,0.5,0.0,4!robot:0.1,0.1,0.0!goal:d1,1.0,1.0,0.0,4!assign:b1,d1\n";
    }
}

int main()
{
    const std::string fixture_name = "test_parse_instance_ws_fixture.txt";
    const std::string fixture_abs_path =
        std::string(CMAKE_SOURCE_DIR) + "/input/" + fixture_name;
    write_fixture_file(fixture_abs_path);

    // Case 1: ws section present, first in the line.
    {
        ObjectMap objects;
        GoalMap goals;
        std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
        std::vector<ReloPush::State> robots;
        bool has_ws = false;
        double ws_x = -1.0, ws_y = -1.0;

        bool ok = parse_instance_from_file(fixture_name, 0, objects, goals, robots,
                                            objGoalPairs, &has_ws, &ws_x, &ws_y);
        assert(ok);
        assert(has_ws);
        assert(std::fabs(ws_x - 4.5) < 1e-9);
        assert(std::fabs(ws_y - 4.5) < 1e-9);
        assert(objects.count("b1") == 1);
    }

    // Case 2: ws section present, last in the line.
    {
        ObjectMap objects;
        GoalMap goals;
        std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
        std::vector<ReloPush::State> robots;
        bool has_ws = false;
        double ws_x = -1.0, ws_y = -1.0;

        bool ok = parse_instance_from_file(fixture_name, 1, objects, goals, robots,
                                            objGoalPairs, &has_ws, &ws_x, &ws_y);
        assert(ok);
        assert(has_ws);
        assert(std::fabs(ws_x - 2.25) < 1e-9);
        assert(std::fabs(ws_y - 2.25) < 1e-9);
    }

    // Case 3: no ws section -> has_ws stays false, ws_x/ws_y untouched.
    {
        ObjectMap objects;
        GoalMap goals;
        std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
        std::vector<ReloPush::State> robots;
        bool has_ws = true;      // pre-set to non-default to prove it gets reset to false
        double ws_x = 123.0;     // sentinel: must remain unchanged
        double ws_y = 456.0;     // sentinel: must remain unchanged

        bool ok = parse_instance_from_file(fixture_name, 2, objects, goals, robots,
                                            objGoalPairs, &has_ws, &ws_x, &ws_y);
        assert(ok);
        assert(!has_ws);
        assert(ws_x == 123.0);
        assert(ws_y == 456.0);
    }

    // Case 4: legacy 6-arg call (no ws out-params) still compiles and works,
    // matching the call signature used in test/test_parse_instance.cpp.
    {
        ObjectMap objects;
        GoalMap goals;
        std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
        std::vector<ReloPush::State> robots;

        bool ok = parse_instance_from_file(fixture_name, 2, objects, goals, robots, objGoalPairs);
        assert(ok);
        assert(objects.count("b1") == 1);
    }

    std::remove(fixture_abs_path.c_str());

    std::cout << "test_parse_instance_ws: all cases passed" << std::endl;
    return 0;
}
