#include <ReloPush/FinalSequenceHandoff.h>

#include <ReloPush/SerializeFinalSequence.h>
#include <ReloPush/base64.h>
#include <zmq.hpp>

#include <stdexcept>
#include <utility>

namespace ReloPush
{
namespace
{
std::vector<std::string> splitMessageParts(
    const std::string &text,
    char delimiter)
{
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (start <= text.size())
    {
        const std::size_t pos = text.find(delimiter, start);
        if (pos == std::string::npos)
        {
            parts.push_back(text.substr(start));
            break;
        }

        parts.push_back(text.substr(start, pos - start));
        start = pos + 1;
    }
    return parts;
}

std::string encodeTextBase64(const std::string &text)
{
    return base64_encode(
        reinterpret_cast<const unsigned char *>(text.data()),
        text.size());
}

bool decodeTextBase64(
    const std::string &encoded_text,
    std::string &decoded_text_out,
    std::string *error_message)
{
    try
    {
        decoded_text_out = base64_decode(encoded_text, false);
        return true;
    }
    catch (const std::exception &ex)
    {
        if (error_message != nullptr)
        {
            *error_message = ex.what();
        }
        decoded_text_out.clear();
        return false;
    }
}

std::string encodeInstanceInfoSegment(const HandoffInstanceInfo &instance_info)
{
    return encodeTextBase64(instance_info.file_name) + "|" +
           std::to_string(instance_info.instance_index);
}

bool decodeInstanceInfoSegment(
    const std::string &encoded_segment,
    HandoffInstanceInfo &instance_info_out,
    std::string *error_message)
{
    const std::vector<std::string> parts = splitMessageParts(encoded_segment, '|');
    if (parts.size() != 2)
    {
        if (error_message != nullptr)
        {
            *error_message = "instance info header did not contain exactly 2 fields";
        }
        return false;
    }

    std::string decoded_file_name;
    if (!decodeTextBase64(parts[0], decoded_file_name, error_message))
    {
        return false;
    }

    try
    {
        instance_info_out.file_name = decoded_file_name;
        instance_info_out.instance_index = std::stoi(parts[1]);
        return true;
    }
    catch (const std::exception &ex)
    {
        if (error_message != nullptr)
        {
            *error_message = ex.what();
        }
        return false;
    }
}

std::string buildDetailMessage(
    const char *prefix,
    const std::string &detail)
{
    if (detail.empty())
    {
        return std::string(prefix);
    }

    return std::string(prefix) + detail;
}
} // namespace

struct FinalSequenceHandoffClient::Impl
{
    zmq::context_t context{1};
    zmq::socket_t socket{context, zmq::socket_type::req};
    bool connected = false;

    Impl()
    {
        socket.set(zmq::sockopt::linger, 0);
    }
};

struct FinalSequenceHandoffServer::Impl
{
    zmq::context_t context{1};
    zmq::socket_t socket{context, zmq::socket_type::rep};
    bool bound = false;
    bool pending_request = false;

    Impl()
    {
        socket.set(zmq::sockopt::linger, 0);
    }
};

std::string encodeFinalSequenceBase64(
    const std::vector<FinalAllocation> &final_sequence)
{
    const std::string binary_data = serializeFinalSequence(final_sequence);
    return base64_encode(
        reinterpret_cast<const unsigned char *>(binary_data.data()),
        binary_data.size());
}

bool decodeFinalSequenceBase64(
    const std::string &encoded_payload,
    std::vector<FinalAllocation> &final_sequence_out,
    std::string *error_message)
{
    try
    {
        const std::string binary_data = base64_decode(encoded_payload, false);
        final_sequence_out = deserializeFinalSequence(binary_data);
        return true;
    }
    catch (const std::exception &ex)
    {
        if (error_message != nullptr)
        {
            *error_message = ex.what();
        }
        final_sequence_out.clear();
        return false;
    }
}

std::string makeFinalSequenceRequest(
    const HandoffInstanceInfo &instance_info,
    const std::vector<FinalAllocation> &final_sequence)
{
    return std::string(kFinalSequenceRequestPrefix) +
           encodeInstanceInfoSegment(instance_info) + "|" +
           encodeFinalSequenceBase64(final_sequence);
}

bool parseFinalSequenceRequest(
    const std::string &request_message,
    HandoffInstanceInfo &instance_info_out,
    std::vector<FinalAllocation> &final_sequence_out,
    std::string *error_message)
{
    if (request_message.rfind(kFinalSequenceRequestPrefix, 0) != 0)
    {
        if (error_message != nullptr)
        {
            *error_message = "request did not start with FINAL_SEQUENCE_B64 prefix";
        }
        final_sequence_out.clear();
        return false;
    }

    const std::string encoded_payload =
        request_message.substr(std::string(kFinalSequenceRequestPrefix).size());

    const std::vector<std::string> parts = splitMessageParts(encoded_payload, '|');
    if (parts.size() != 3)
    {
        if (error_message != nullptr)
        {
            *error_message = "final-sequence request did not contain metadata and payload";
        }
        final_sequence_out.clear();
        return false;
    }

    if (!decodeInstanceInfoSegment(parts[0] + "|" + parts[1], instance_info_out, error_message))
    {
        final_sequence_out.clear();
        return false;
    }

    return decodeFinalSequenceBase64(
        parts[2],
        final_sequence_out,
        error_message);
}

std::string makeAbortRequest(
    const HandoffInstanceInfo &instance_info,
    const std::string &reason)
{
    return std::string(kAbortRequestPrefix) +
           encodeInstanceInfoSegment(instance_info) + "|" +
           encodeTextBase64(reason);
}

bool parseAbortRequest(
    const std::string &request_message,
    HandoffInstanceInfo *instance_info_out,
    std::string *reason_out)
{
    if (request_message.rfind(kAbortRequestPrefix, 0) != 0)
    {
        return false;
    }

    const std::string encoded_payload =
        request_message.substr(std::string(kAbortRequestPrefix).size());
    const std::vector<std::string> parts = splitMessageParts(encoded_payload, '|');
    if (parts.size() != 3)
    {
        return false;
    }

    HandoffInstanceInfo parsed_instance_info;
    if (!decodeInstanceInfoSegment(parts[0] + "|" + parts[1], parsed_instance_info, nullptr))
    {
        return false;
    }

    std::string decoded_reason;
    if (!decodeTextBase64(parts[2], decoded_reason, nullptr))
    {
        return false;
    }

    if (instance_info_out != nullptr)
    {
        *instance_info_out = parsed_instance_info;
    }
    if (reason_out != nullptr)
    {
        *reason_out = decoded_reason;
    }
    return true;
}

std::string makeMarsReply(bool success, const std::string &detail)
{
    return success
               ? buildDetailMessage(kMarsReplyOkPrefix, detail)
               : buildDetailMessage(kMarsReplyErrorPrefix, detail);
}

bool isMarsSuccessReply(const std::string &reply_message)
{
    return reply_message.rfind(kMarsReplyOkPrefix, 0) == 0;
}

FinalSequenceHandoffClient::FinalSequenceHandoffClient()
    : impl_(std::make_unique<Impl>())
{
}

FinalSequenceHandoffClient::~FinalSequenceHandoffClient() = default;

FinalSequenceHandoffClient::FinalSequenceHandoffClient(
    FinalSequenceHandoffClient &&other) noexcept = default;

FinalSequenceHandoffClient &FinalSequenceHandoffClient::operator=(
    FinalSequenceHandoffClient &&other) noexcept = default;

void FinalSequenceHandoffClient::connect(const std::string &endpoint)
{
    if (impl_ == nullptr)
    {
        throw std::runtime_error("FinalSequenceHandoffClient is not initialized");
    }

    impl_->socket.connect(endpoint);
    impl_->connected = true;
}

std::string FinalSequenceHandoffClient::sendRequestAndWaitForReply(
    const std::string &request_message)
{
    if (impl_ == nullptr || !impl_->connected)
    {
        throw std::runtime_error("FinalSequenceHandoffClient must connect before sending");
    }

    const auto send_result =
        impl_->socket.send(zmq::buffer(request_message), zmq::send_flags::none);
    if (!send_result)
    {
        throw std::runtime_error("failed to send request to MARS");
    }

    zmq::message_t reply;
    const auto recv_result = impl_->socket.recv(reply, zmq::recv_flags::none);
    if (!recv_result)
    {
        throw std::runtime_error("failed to receive reply from MARS");
    }

    return std::string(
        static_cast<const char *>(reply.data()),
        reply.size());
}

std::string FinalSequenceHandoffClient::sendFinalSequenceAndWaitForReply(
    const HandoffInstanceInfo &instance_info,
    const std::vector<FinalAllocation> &final_sequence)
{
    return sendRequestAndWaitForReply(
        makeFinalSequenceRequest(instance_info, final_sequence));
}

std::string FinalSequenceHandoffClient::sendAbortAndWaitForReply(
    const HandoffInstanceInfo &instance_info,
    const std::string &reason)
{
    return sendRequestAndWaitForReply(makeAbortRequest(instance_info, reason));
}

FinalSequenceHandoffServer::FinalSequenceHandoffServer()
    : impl_(std::make_unique<Impl>())
{
}

FinalSequenceHandoffServer::~FinalSequenceHandoffServer() = default;

FinalSequenceHandoffServer::FinalSequenceHandoffServer(
    FinalSequenceHandoffServer &&other) noexcept = default;

FinalSequenceHandoffServer &FinalSequenceHandoffServer::operator=(
    FinalSequenceHandoffServer &&other) noexcept = default;

void FinalSequenceHandoffServer::bind(const std::string &endpoint)
{
    if (impl_ == nullptr)
    {
        throw std::runtime_error("FinalSequenceHandoffServer is not initialized");
    }

    impl_->socket.bind(endpoint);
    impl_->bound = true;
}

std::string FinalSequenceHandoffServer::waitForRequest()
{
    if (impl_ == nullptr || !impl_->bound)
    {
        throw std::runtime_error("FinalSequenceHandoffServer must bind before receiving");
    }

    zmq::message_t request;
    const auto recv_result = impl_->socket.recv(request, zmq::recv_flags::none);
    if (!recv_result)
    {
        throw std::runtime_error("failed to receive request from ReloPush");
    }

    impl_->pending_request = true;
    return std::string(
        static_cast<const char *>(request.data()),
        request.size());
}

void FinalSequenceHandoffServer::sendReply(const std::string &reply_message)
{
    if (impl_ == nullptr || !impl_->pending_request)
    {
        throw std::runtime_error("no pending request is waiting for a reply");
    }

    const auto send_result =
        impl_->socket.send(zmq::buffer(reply_message), zmq::send_flags::none);
    if (!send_result)
    {
        throw std::runtime_error("failed to send reply to ReloPush");
    }

    impl_->pending_request = false;
}

bool FinalSequenceHandoffServer::hasPendingRequest() const
{
    return impl_ != nullptr && impl_->pending_request;
}
} // namespace ReloPush
