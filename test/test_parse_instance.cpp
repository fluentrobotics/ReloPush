#include <batchInstanceParcer.hpp>
#include <ObjectInfo.hpp>



int main(int argc, char *argv[])
{
    std::string file_name = "test_batch.txt";
    int instance_ind = 2;
    bool use_opt = true;

    // parsing info

    ObjectMap objects;
    GoalMap   goals;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
    std::vector<ReloPush::State> robots;

    handle_args(argc, argv, file_name, instance_ind, use_opt);

    // test parse
    parse_instance_from_file(file_name, instance_ind, objects, goals, robots, objGoalPairs);

    return 0;
}
