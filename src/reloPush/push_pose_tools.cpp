#include <reloPush/push_pose_tools.h>

State find_pre_push(State& goalState, float distance)
{
    State outState(goalState);

    // Calculate the new x and y coordinates
    outState.x -= distance * cos(goalState.yaw);
    outState.y -= distance * sin(goalState.yaw);

    // change angle range
    outState.yaw = jeeho::convertEulerRange_to_2pi(outState.yaw);

    return outState;
}

State find_post_push(State& goalState, float distance)
{
    State outState(goalState);

    // Calculate the new x and y coordinates
    outState.x += distance * cos(goalState.yaw);
    outState.y += distance * sin(goalState.yaw);

    return outState;
}