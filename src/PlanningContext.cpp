#include <PlanningContext.hpp>

ReloPush::State object_to_state(ObjectInfo& obj)
{
    return ReloPush::State(obj.x,obj.y,obj.nominalOrientation);
}

ReloPush::State goal_to_state(GoalInfo& goal)
{
    return ReloPush::State(goal.x,goal.y,goal.nominalOrientation);
}

// Function to convert ObjectMap to std::vector<State>
std::vector<ReloPush::State> convert_to_states(ObjectMap &objects) {
    std::vector<ReloPush::State> states;
    for (auto &pair : objects) {
        states.push_back(object_to_state(pair.second));
    }
    return states;
}

// Function to convert GoalMap to std::vector<State>
std::vector<ReloPush::State> convert_to_states(GoalMap &objects) {
    std::vector<ReloPush::State> states;
    for (auto &pair : objects) {
        states.push_back(goal_to_state(pair.second));
    }
    return states;
}
