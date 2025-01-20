#ifndef PUSH_POSE_TOOLS_H
#define PUSH_POSE_TOOLS_H

#include <State.h>
#include <TfTools.h>
#include <Parameters.hpp>

ReloPush::State find_pre_push(ReloPush::State& goalState, float distance = params::pre_push_dist);
ReloPush::State find_post_push(ReloPush::State& goalState, float distance = params::pre_push_dist);

#endif
