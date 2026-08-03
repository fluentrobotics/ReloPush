#pragma once

#include <PHAstarPushDemoOptions.h>
#include <PHAstarPushDemoTypes.h>

#include <string>

// MARS-side handoff gate to a separately running mars_sim_viz process (Phase
// A of the sim-viz handoff design). Instead of MARS driving robots/
// simulators in-process (see run_on_robots_pipeline in
// MARS/src/SearchOrchestrator.cpp), a completed, all-tasks-succeeded run can
// be handed off to a visualizer that owns execution from that point on.
enum class SimVizHandoffResult
{
  // options.sim_viz_handoff was false; no network activity happened at all.
  NotEnabled,
  // sim_viz_handoff was true but the visualizer did not answer PING (not
  // running, wrong endpoint, etc). No file was written.
  VizAbsent,
  // The visualizer PONGed but something after that failed: writing the
  // .scn.b64 file, the EXECUTE round-trip timing out, or an explicit ERR
  // reply.
  HandoffFailed,
  // The visualizer accepted the handoff (ACK_EXECUTE). The visualizer now
  // owns execution; the caller does nothing further.
  HandoffOk,
};

const char *sim_viz_handoff_result_name(SimVizHandoffResult result);

// Attempts to hand `executed` off to a running mars_sim_viz instance:
//   1. options.sim_viz_handoff must be true (else NotEnabled immediately --
//      no socket is ever created).
//   2. PING options.sim_viz_endpoint (fresh REQ socket per attempt, ~1s
//      rcvtimeo, up to 2 attempts -- see src/ReloPush/FinalSequenceHandoff.cpp
//      for the sibling rcvtimeo pattern this mirrors; a fresh socket per
//      attempt is required here rather than reusing/retrying one socket
//      because a REQ socket that times out mid-recv is left mid-transaction
//      and cannot safely send again). No PONG within that budget -> logs
//      "[SimViz] visualizer not running; terminating without simulation"
//      and returns VizAbsent.
//   3. On PONG: serializes `executed` (via serialize_executed_scenario_b64,
//      MARS/include/ExecutedScenarioSerialization.h) to
//      "<options.sim_handoff_out_dir>/<sanitized label>.scn.b64" (directory
//      created if missing; `label` is sanitized the same way
//      run_on_robots_pipeline's CSV paths already are -- see
//      CsvLogging.h's sanitize_filename_component), then sends
//      "EXECUTE <absolute path>" (fresh REQ socket, ~5s rcvtimeo, single
//      attempt) and waits for a reply starting with "ACK_EXECUTE".
//      Anything else (ERR reply, timeout, file-write failure) is logged and
//      returns HandoffFailed.
//   4. "ACK_EXECUTE" -> HandoffOk. The visualizer now owns execution.
//
// Never blocks unboundedly: every network wait is bounded by an rcvtimeo, so
// this always returns within a few seconds even when nothing is listening on
// options.sim_viz_endpoint.
SimVizHandoffResult maybe_handoff_to_sim_viz(
    const RuntimeOptions &options,
    const ExecutedScenario &executed,
    const std::string &label);
