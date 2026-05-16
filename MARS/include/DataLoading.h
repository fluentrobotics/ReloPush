#ifndef DATA_LOADING_H
#define DATA_LOADING_H

#include <PHAstarPushDemoOptions.h>
#include <ReloPush/FinalSequenceHandoff.h>
#include <ReloPush/TaskAllocation.hpp>
#include <vector>
#include <memory>

// Load data from either ReloPush handoff or file-based sequence
bool load_data(
    const RuntimeOptions &options,
    ReloPush::HandoffInstanceInfo &instance_info,
    std::vector<FinalAllocation> &loaded_sequence,
    std::unique_ptr<ReloPush::FinalSequenceHandoffServer> &handoff_server);

#endif // DATA_LOADING_H
