#include <PlanningContext.hpp>

State object_to_state(ObjectInfo& obj)
{
    return State(obj.x,obj.y,obj.nominalOrientation);
}

State goal_to_state(GoalInfo& goal)
{
    return State(goal.x,goal.y,goal.nominalOrientation);
}

// Function to convert ObjectMap to std::vector<State>
std::vector<State> convert_to_states(ObjectMap &objects) {
    std::vector<State> states;
    for (auto &pair : objects) {
        states.push_back(object_to_state(pair.second));
    }
    return states;
}

// Function to convert GoalMap to std::vector<State>
std::vector<State> convert_to_states(GoalMap &objects) {
    std::vector<State> states;
    for (auto &pair : objects) {
        states.push_back(goal_to_state(pair.second));
    }
    return states;
}
