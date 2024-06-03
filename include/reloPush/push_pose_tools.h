#ifndef PUSH_POSE_TOOLS_H
#define PUSH_POSE_TOOLS_H

#include <omplTools/State.h>
#include <pathPlanTools/tf_tools.h>

State find_pre_push(State& goalState, float distance = 0.6f);
State find_post_push(State& goalState, float distance = 0.6f);

#endif