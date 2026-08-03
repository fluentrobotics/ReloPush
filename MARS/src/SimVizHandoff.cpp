#include <SimVizHandoff.h>

#include <ExecutedScenarioSerialization.h>
#include <CsvLogging.h> // sanitize_filename_component

#include <zmq.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace
{
constexpr int kPingTimeoutMs = 1000;
constexpr int kPingMaxAttempts = 2;
constexpr int kExecuteTimeoutMs = 5000;

// Sends `message` to `endpoint` over a freshly-created REQ socket and waits
// up to `timeout_ms` for a reply. Returns true + fills `reply_out` on
// success (a reply was received); false on any failure (connect error, send
// timeout, recv timeout). A fresh socket per call is intentional -- see
// SimVizHandoff.h's doc comment on maybe_handoff_to_sim_viz.
bool send_and_wait(const std::string &endpoint, const std::string &message,
                    int timeout_ms, std::string &reply_out)
{
  try
  {
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::req);
    sock.set(zmq::sockopt::linger, 0);
    sock.set(zmq::sockopt::rcvtimeo, timeout_ms);
    sock.set(zmq::sockopt::sndtimeo, timeout_ms);
    sock.set(zmq::sockopt::connect_timeout, timeout_ms);
    sock.connect(endpoint);

    const auto send_result = sock.send(zmq::buffer(message), zmq::send_flags::none);
    if (!send_result)
      return false;

    zmq::message_t reply;
    const auto recv_result = sock.recv(reply, zmq::recv_flags::none);
    if (!recv_result)
      return false;

    reply_out.assign(static_cast<const char *>(reply.data()), reply.size());
    return true;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[SimViz] ZMQ error talking to " << endpoint << ": " << ex.what()
              << std::endl;
    return false;
  }
}

bool ping_sim_viz(const std::string &endpoint)
{
  for (int attempt = 0; attempt < kPingMaxAttempts; ++attempt)
  {
    std::string reply;
    if (send_and_wait(endpoint, "PING", kPingTimeoutMs, reply) &&
        reply.rfind("PONG", 0) == 0)
    {
      return true;
    }
  }
  return false;
}
} // namespace

const char *sim_viz_handoff_result_name(SimVizHandoffResult result)
{
  switch (result)
  {
  case SimVizHandoffResult::NotEnabled:
    return "NotEnabled";
  case SimVizHandoffResult::VizAbsent:
    return "VizAbsent";
  case SimVizHandoffResult::HandoffFailed:
    return "HandoffFailed";
  case SimVizHandoffResult::HandoffOk:
    return "HandoffOk";
  }
  return "Unknown";
}

SimVizHandoffResult maybe_handoff_to_sim_viz(
    const RuntimeOptions &options,
    const ExecutedScenario &executed,
    const std::string &label)
{
  if (!options.sim_viz_handoff)
    return SimVizHandoffResult::NotEnabled;

  std::cout << "[SimViz] Pinging visualizer at " << options.sim_viz_endpoint << "..."
            << std::endl;
  if (!ping_sim_viz(options.sim_viz_endpoint))
  {
    std::cout << "[SimViz] visualizer not running; terminating without simulation"
              << std::endl;
    return SimVizHandoffResult::VizAbsent;
  }

  std::error_code mkdir_ec;
  std::filesystem::create_directories(options.sim_handoff_out_dir, mkdir_ec);
  if (mkdir_ec)
  {
    std::cerr << "[SimViz] Failed to create output directory '"
              << options.sim_handoff_out_dir << "': " << mkdir_ec.message() << std::endl;
    return SimVizHandoffResult::HandoffFailed;
  }

  const std::filesystem::path scn_path =
      std::filesystem::path(options.sim_handoff_out_dir) /
      (sanitize_filename_component(label) + ".scn.b64");

  {
    std::ofstream scn_out(scn_path, std::ios::binary);
    if (!scn_out.is_open())
    {
      std::cerr << "[SimViz] Failed to open '" << scn_path.string()
                << "' for writing" << std::endl;
      return SimVizHandoffResult::HandoffFailed;
    }
    scn_out << serialize_executed_scenario_b64(executed);
    if (!scn_out.good())
    {
      std::cerr << "[SimViz] Failed writing '" << scn_path.string() << "'" << std::endl;
      return SimVizHandoffResult::HandoffFailed;
    }
  }

  std::error_code abs_ec;
  std::filesystem::path abs_path = std::filesystem::absolute(scn_path, abs_ec);
  if (abs_ec)
    abs_path = scn_path;

  const std::string execute_message = "EXECUTE " + abs_path.string();
  std::cout << "[SimViz] Sending " << execute_message << std::endl;

  std::string reply;
  if (!send_and_wait(options.sim_viz_endpoint, execute_message, kExecuteTimeoutMs, reply))
  {
    std::cerr << "[SimViz] No response to EXECUTE (timeout); terminating without simulation"
              << std::endl;
    return SimVizHandoffResult::HandoffFailed;
  }

  if (reply.rfind("ACK_EXECUTE", 0) != 0)
  {
    std::cerr << "[SimViz] visualizer rejected EXECUTE: " << reply << std::endl;
    return SimVizHandoffResult::HandoffFailed;
  }

  std::cout << "[SimViz] Handoff accepted by visualizer. MARS terminating; visualizer "
               "owns execution."
            << std::endl;
  return SimVizHandoffResult::HandoffOk;
}
