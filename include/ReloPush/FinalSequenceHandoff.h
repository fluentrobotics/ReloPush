#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ReloPush/TaskAllocation.hpp>

namespace ReloPush
{
inline constexpr const char *kDefaultMarsHandoffEndpoint = "tcp://127.0.0.1:5566";
inline constexpr const char *kFinalSequenceRequestPrefix = "FINAL_SEQUENCE_B64:";
inline constexpr const char *kAbortRequestPrefix = "RELOPUSH_ABORT:";
inline constexpr const char *kMarsReplyOkPrefix = "MARS_DONE:";
inline constexpr const char *kMarsReplyErrorPrefix = "MARS_ERROR:";

struct HandoffInstanceInfo
{
    std::string file_name;
    int instance_index = -1;
};

std::string encodeFinalSequenceBase64(
    const std::vector<FinalAllocation> &final_sequence);

bool decodeFinalSequenceBase64(
    const std::string &encoded_payload,
    std::vector<FinalAllocation> &final_sequence_out,
    std::string *error_message = nullptr);

std::string makeFinalSequenceRequest(
    const HandoffInstanceInfo &instance_info,
    const std::vector<FinalAllocation> &final_sequence);

bool parseFinalSequenceRequest(
    const std::string &request_message,
    HandoffInstanceInfo &instance_info_out,
    std::vector<FinalAllocation> &final_sequence_out,
    std::string *error_message = nullptr);

std::string makeAbortRequest(
    const HandoffInstanceInfo &instance_info,
    const std::string &reason);

bool parseAbortRequest(
    const std::string &request_message,
    HandoffInstanceInfo *instance_info_out = nullptr,
    std::string *reason_out = nullptr);

std::string makeMarsReply(bool success, const std::string &detail = "");

bool isMarsSuccessReply(const std::string &reply_message);

class FinalSequenceHandoffClient
{
public:
    FinalSequenceHandoffClient();
    ~FinalSequenceHandoffClient();

    FinalSequenceHandoffClient(FinalSequenceHandoffClient &&other) noexcept;
    FinalSequenceHandoffClient &operator=(FinalSequenceHandoffClient &&other) noexcept;

    FinalSequenceHandoffClient(const FinalSequenceHandoffClient &) = delete;
    FinalSequenceHandoffClient &operator=(const FinalSequenceHandoffClient &) = delete;

    void connect(const std::string &endpoint = kDefaultMarsHandoffEndpoint);
    std::string sendRequestAndWaitForReply(const std::string &request_message);
    std::string sendFinalSequenceAndWaitForReply(
        const HandoffInstanceInfo &instance_info,
        const std::vector<FinalAllocation> &final_sequence);
    std::string sendAbortAndWaitForReply(
        const HandoffInstanceInfo &instance_info,
        const std::string &reason);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class FinalSequenceHandoffServer
{
public:
    FinalSequenceHandoffServer();
    ~FinalSequenceHandoffServer();

    FinalSequenceHandoffServer(FinalSequenceHandoffServer &&other) noexcept;
    FinalSequenceHandoffServer &operator=(FinalSequenceHandoffServer &&other) noexcept;

    FinalSequenceHandoffServer(const FinalSequenceHandoffServer &) = delete;
    FinalSequenceHandoffServer &operator=(const FinalSequenceHandoffServer &) = delete;

    void bind(const std::string &endpoint = kDefaultMarsHandoffEndpoint);
    std::string waitForRequest();
    void sendReply(const std::string &reply_message);
    bool hasPendingRequest() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace ReloPush
