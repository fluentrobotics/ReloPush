#ifndef PUSH_POSE_TOOLS_H
#define PUSH_POSE_TOOLS_H

#include <omplTools/State.h>
#include <pathPlanTools/tf_tools.h>
#include <reloPush/params.h>

State find_pre_push(State& goalState, float distance = params::pre_push_dist);
State find_post_push(State& goalState, float distance = params::pre_push_dist);

#endif