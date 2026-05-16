#include <DataLoading.h>
#include <RuntimeOptionsParsing.h>
#include <ReloPushSetup.h>
#include <ReloPush/SerializeFinalSequence.h>
#include <ReloPush/base64.h>
#include <config.h>
#include <fstream>
#include <iterator>
#include <iostream>

namespace
{
std::vector<FinalAllocation> load_final_sequence_from_file(
    const std::string &filename)
{
  std::ifstream in_file(filename);
  if (!in_file)
  {
    std::cerr << "Error: Could not open file " << filename
              << " for reading." << std::endl;
    return {};
  }

  std::string base64_data((std::istreambuf_iterator<char>(in_file)),
                          std::istreambuf_iterator<char>());
  const std::string binary_data = base64_decode(base64_data);
  return deserializeFinalSequence(binary_data);
}
} // namespace

bool load_data(
    const RuntimeOptions &options,
    ReloPush::HandoffInstanceInfo &instance_info,
    std::vector<FinalAllocation> &loaded_sequence,
    std::unique_ptr<ReloPush::FinalSequenceHandoffServer> &handoff_server)
{
  if (options.integrated_mode)
  {
    try
    {
      handoff_server = std::make_unique<ReloPush::FinalSequenceHandoffServer>();
      handoff_server->bind(options.handoff_endpoint);

      std::cout << "[Integration] Waiting for ReloPush handoff on "
                << options.handoff_endpoint << std::endl;

      const std::string request_message = handoff_server->waitForRequest();

      std::string abort_reason;
      if (ReloPush::parseAbortRequest(request_message, &instance_info, &abort_reason))
      {
        if (abort_reason.empty())
        {
          abort_reason = "ReloPush aborted before sending a final sequence";
        }

        std::cerr << "[Integration] " << abort_reason << std::endl;
        handoff_server->sendReply(ReloPush::makeMarsReply(false, abort_reason));
        return false;
      }

      std::string error_message;
      if (!ReloPush::parseFinalSequenceRequest(
              request_message,
              instance_info,
              loaded_sequence,
              &error_message))
      {
        if (error_message.empty())
        {
          error_message = "Failed to decode the handed-off final sequence";
        }

        std::cerr << "[Integration] " << error_message << std::endl;
        handoff_server->sendReply(ReloPush::makeMarsReply(false, error_message));
        return false;
      }

      if (loaded_sequence.empty())
      {
        const std::string detail =
            "Received an empty final sequence from ReloPush";
        std::cerr << "[Integration] " << detail << std::endl;
        handoff_server->sendReply(ReloPush::makeMarsReply(false, detail));
        return false;
      }

      std::cout << "[Integration] Received " << loaded_sequence.size()
                << " tasks from ReloPush." << std::endl;
      return true;
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[Integration] Failed to receive handoff: "
                << ex.what() << std::endl;
      if (handoff_server && handoff_server->hasPendingRequest())
      {
        handoff_server->sendReply(
            ReloPush::makeMarsReply(false, ex.what()));
      }
      return false;
    }
  }

  instance_info = default_instance_info(options);
  std::string filename = options.input_sequence_path.empty()
                             ? default_sequence_path()
                             : options.input_sequence_path;
  std::cout << "[System] Loading sequence: " << filename << std::endl;

  loaded_sequence = load_final_sequence_from_file(filename);
  if (loaded_sequence.empty())
  {
    std::cerr << "[System] Failed to load sequence." << std::endl;
    return false;
  }

  return true;
}
