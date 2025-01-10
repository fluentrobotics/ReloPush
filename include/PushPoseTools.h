#ifndef PUSH_POSE_TOOLS_H
#define PUSH_POSE_TOOLS_H

#include <State.h>
#include <TfTools.h>>
#include <Parameters.hpp>>

State find_pre_push(State& goalState, float distance = params::pre_push_dist);
State find_post_push(State& goalState, float distance = params::pre_push_dist);

#endif
