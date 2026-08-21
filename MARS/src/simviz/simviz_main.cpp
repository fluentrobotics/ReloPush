// mars_sim_viz entry point. Phase B2: GUI (SimVizWindow) is the default;
// --headless preserves the Phase B1 window-free manager (QCoreApplication +
// bare QTimer), which is what the automated Phase B tests drive. Every flag
// below is this binary's OWN CLI surface, independent of MARS's
// RuntimeOptions -- the defaults mirror PHAstarPushDemoOptions.h's
// robot_controller_port_start/mpc_vesc_port_start/mpc_localization_port_start
// so a default `mars_sim_viz` + a default MARS run agree without any flags
// on either side.

#include "SimVizCore.h"
#include "SimVizWindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QFileInfo>
#include <QSocketNotifier>
#include <QString>
#include <QTimer>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <csignal>
#include <sys/socket.h>
#include <unistd.h>

// Required global -- see PHAstar/PHAstar.h's `extern bool DEBUG_VIS;`. Every
// MARS executable that links the shared PlanningHelpers.cpp/SafeParking.cpp/
// AllocationSearch.cpp/TaskExecution.cpp object set (mars_sim_viz included,
// even though it never calls into the search engine itself) must define this
// exactly once; mirrors MARS/src/PHAstar_push_demo.cpp and every MARS test
// binary.
bool DEBUG_VIS = false;

namespace
{

bool starts_with(const std::string &s, const std::string &prefix)
{
  return s.rfind(prefix, 0) == 0;
}

// -----------------------------------------------------------------------
// SIGTERM/SIGINT handling.
//
// Without this, killing mars_sim_viz (operator Ctrl+C, `systemctl stop`, a
// supervisor restart, ...) leaves every spawned mpc_robot_sim/mpc_controller
// child running as an orphan, still bound to its TCP ports -- the process is
// terminated by the kernel's default SIGTERM disposition before the C++
// destructor chain (ExecutionManager::~ExecutionManager -> abort() ->
// terminate_children()) ever runs.
//
// A raw signal handler cannot safely touch QProcess/ZMQ/mutexes (not
// async-signal-safe), so we use the standard Qt self-pipe pattern: the
// handler only writes one byte to a socketpair (write() is async-signal-
// safe), and a QSocketNotifier on the Qt event loop's thread reads it and
// performs the actual teardown (ExecutionManager::abort() tears down ALL
// children, mirroring SearchOrchestrator.cpp's mpc_signal_handler /
// cleanup_mpc_processes for the existing --run-on-robots path).
int g_signal_fd[2] = {-1, -1};

extern "C" void handle_unix_signal(int)
{
  const char one = 1;
  // Best-effort; if this write fails there is nothing safe left to do inside
  // a signal handler.
  ssize_t written = ::write(g_signal_fd[0], &one, sizeof(one));
  (void)written;
}

bool install_signal_handlers()
{
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, g_signal_fd) != 0)
  {
    std::cerr << "[mars_sim_viz] Warning: socketpair() failed; SIGTERM/SIGINT will fall back "
                 "to default (abrupt) termination -- spawned children may be orphaned."
              << std::endl;
    return false;
  }

  struct sigaction sa;
  sa.sa_handler = handle_unix_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGINT, &sa, nullptr);
  return true;
}

// Reads (and discards) the notifier byte, tears down all of `manager`'s
// spawned children, then quits the Qt event loop. Shared by headless and GUI
// mode.
void handle_shutdown_signal(simviz::SimVizManager &manager)
{
  char buf;
  ssize_t n = ::read(g_signal_fd[1], &buf, sizeof(buf));
  (void)n;
  std::cerr << "\n[mars_sim_viz] Caught SIGTERM/SIGINT -- tearing down spawned child processes..."
            << std::endl;
  manager.execution_manager().abort();
  QCoreApplication::quit();
}

void print_usage()
{
  std::cout <<
      "mars_sim_viz -- headless/GUI visualizer + child-process manager for MARS\n"
      "sim-viz handoff runs.\n\n"
      "Options:\n"
      "  --headless                    Run without a GUI window (what the automated\n"
      "                                 Phase B tests drive). Default is the GUI\n"
      "                                 (SimVizWindow) mode.\n"
      "  --control-port=N               REP control socket port (default 5601)\n"
      "  --handshake-port-start=N       mpc_controller handshake base port (default 11110)\n"
      "  --vesc-port-start=N            mpc_robot_sim --cmd-endpoint base port (default 3160)\n"
      "  --loc-port-start=N             mpc_robot_sim --loc-endpoint base port (default 3260)\n"
      "  --mpc-controller-path=PATH     Override mpc_controller binary path\n"
      "  --robot-sim-path=PATH          Override mpc_robot_sim binary path\n"
      "  --run-dir=PATH                 Base directory for per-run child CSV logs\n"
      "                                 (default results/sim_handoff_runs)\n"
      "  --noise-sigma-pct=N            FEATURE C: ACCEL-channel actuation-noise\n"
      "                                 sigma (fraction of actuator limit,\n"
      "                                 [0,0.25], default 0) applied to every\n"
      "                                 spawned mpc_robot_sim from t=0 and to the\n"
      "                                 ~1Hz live sim_config publish while a run is\n"
      "                                 active. Same value the GUI's \"Motor (accel)\n"
      "                                 noise σ\" toolbar slider controls.\n"
      "  --steer-noise-sigma-pct=N      PART (1) STEERING-NOISE SPLIT: independent\n"
      "                                 STEER-channel actuation-noise sigma, same\n"
      "                                 range/semantics as --noise-sigma-pct= above.\n"
      "                                 Same value the GUI's \"Steering noise σ\"\n"
      "                                 toolbar slider controls.\n"
      "  --noise-seed=N                 FEATURE C (testability only): passthrough\n"
      "                                 --noise-seed= for every spawned\n"
      "                                 mpc_robot_sim. Absent by default.\n"
      "  --deadband                     MOTOR STALL: enable the min-speed deadband\n"
      "                                 plant model on every spawned mpc_robot_sim\n"
      "                                 (--deadband --min-moving-speed=/\n"
      "                                 --min-sustain-speed=) and give each spawned\n"
      "                                 mpc_controller a matching --robot-spec\n"
      "                                 override. Same effect as the GUI's \"Enable\n"
      "                                 stall (min speed)\" checkbox. Default off.\n"
      "  --stall-level=N                 MOTOR STALL: min_moving_speed/\n"
      "                                 min_sustain_speed value (m/s, [0,0.25],\n"
      "                                 default 0.10) used for BOTH thresholds when\n"
      "                                 --deadband is also passed. Same value the\n"
      "                                 GUI's \"Stall level\" spinbox controls.\n"
      "  --execute=PATH                 Load PATH (a .scn.b64 result file) as soon\n"
      "                                 as the manager (and, in GUI mode, the\n"
      "                                 window) is up. HEADLESS mode: starts\n"
      "                                 executing it immediately -- the exact same\n"
      "                                 code path as a control-socket\n"
      "                                 \"EXECUTE <path>\" request. GUI mode: only\n"
      "                                 loads + previews it (robots/objects/goals\n"
      "                                 drawn at their initial poses), exactly like\n"
      "                                 File->Open -- it does NOT auto-start;\n"
      "                                 execution begins once the toolbar Start\n"
      "                                 button is pressed (letting the operator\n"
      "                                 connect/map mocap first). The control-socket\n"
      "                                 \"EXECUTE <path>\" request still auto-starts\n"
      "                                 in GUI mode too, same as headless -- only\n"
      "                                 File->Open / --execute's GUI load path wait\n"
      "                                 for Start. Relative paths are resolved\n"
      "                                 against the current working directory. On\n"
      "                                 load failure: headless mode prints the\n"
      "                                 reason to stderr and exits nonzero; GUI mode\n"
      "                                 shows an error dialog and stays open. The\n"
      "                                 control socket remains fully usable\n"
      "                                 alongside a run started this way.\n"
      "  --exit-on-done                 Only meaningful with --execute (or any run\n"
      "                                 started later over the control socket): in\n"
      "                                 headless mode, exit 0 once the run reaches\n"
      "                                 DONE, or nonzero if it ends in ERR --\n"
      "                                 intended for scripted headless replays. In\n"
      "                                 GUI mode this instead closes the window\n"
      "                                 once the run reaches DONE/ERR (still waits\n"
      "                                 on the operator pressing Start first if\n"
      "                                 --execute only loaded+previewed, per above).\n"
      "                                 Default off (the process/window stays up\n"
      "                                 after the run finishes, so the control\n"
      "                                 socket keeps\n"
      "                                 working / the window keeps showing the\n"
      "                                 result).\n"
      "  --stage-first                  STAGING PHASE: before running the executed\n"
      "                                 scenario, drive each PRESENT robot from its\n"
      "                                 actual current pose to its planner-assumed\n"
      "                                 start pose first (one robot at a time,\n"
      "                                 avoiding the others). Same effect as the\n"
      "                                 GUI's \"Stage first\" checkbox. Default off\n"
      "                                 (unchanged prior behavior).\n"
      "  --staging-pos-tol=N             STAGING PHASE: position tolerance in\n"
      "                                 meters (default 0.05) below which a robot is\n"
      "                                 considered already at its start pose (no\n"
      "                                 staging leg needed).\n"
      "  --staging-yaw-tol=N             STAGING PHASE: yaw tolerance in radians\n"
      "                                 (default 0.15).\n"
      "  --staging-margin=N              STAGING PHASE: extra clearance in meters\n"
      "                                 kept from OTHER robots while planning a\n"
      "                                 staging leg (default 0.15).\n"
      "  --staging-max-retries=N         STAGING PHASE: max replan-and-retry\n"
      "                                 attempts after a leg misses its tolerance\n"
      "                                 before failing the whole EXECUTE (default 2).\n"
      "  --staging-localization-wait-s=N STAGING PHASE: \"hardware mode\" presence\n"
      "                                 freshness budget in seconds (default 3.0) --\n"
      "                                 a robot with no localization pose newer than\n"
      "                                 this is treated as MISSING. Ignored when\n"
      "                                 --staging-test-offsets= is also given.\n"
      "  --staging-test-offsets=SPEC     TEST-ONLY: \"robot1:dx,dy,dyaw;robot2:...\".\n"
      "                                 Robots listed get \"current pose\" =\n"
      "                                 start_pose (+) offset and are treated as\n"
      "                                 PRESENT; every other scenario robot is\n"
      "                                 treated as MISSING. Overrides the\n"
      "                                 localization-based presence check above.\n"
      "  --real-mode                    PART B: spawn ONLY mpc_controller per\n"
      "                                 robot (never mpc_robot_sim) -- \"real\n"
      "                                 hardware\" (or a test stand-in) is\n"
      "                                 expected at each robot's configured\n"
      "                                 endpoints instead. Default off (spawns\n"
      "                                 both, unchanged prior behavior).\n"
      "  --real-robots-config=PATH      PART B: per-robot vesc_endpoint/\n"
      "                                 localization_endpoint overrides for\n"
      "                                 --real-mode (JSON, see MARS/config/\n"
      "                                 real_robots.json). Default resolves\n"
      "                                 MARS/config/real_robots.json relative to\n"
      "                                 this executable's own location (so it's\n"
      "                                 found regardless of the current working\n"
      "                                 directory mars_sim_viz was launched\n"
      "                                 from); an EXPLICIT PATH given here is\n"
      "                                 still resolved against the current\n"
      "                                 working directory, same convention as\n"
      "                                 --run-dir=. A missing default file is\n"
      "                                 silently ignored (every robot falls\n"
      "                                 back to its computed default endpoint),\n"
      "                                 a missing EXPLICIT path is a warning.\n"
      "  --stage-real-pos-tol=N          PART B: \"move real robots to initial\n"
      "                                 poses\" (STAGE_REAL) position tolerance\n"
      "                                 in meters (default 0.05).\n"
      "  --stage-real-yaw-tol=N          PART B: STAGE_REAL yaw tolerance in\n"
      "                                 radians (default 0.15).\n"
      "  --stage-real-leg-margin-s=N     TEST-ONLY: overrides the STAGE_REAL\n"
      "                                 per-leg bounded-timeout margin (traj\n"
      "                                 duration + N seconds, default 15) --\n"
      "                                 lets a test exercise the deadline-\n"
      "                                 exceeded failure path without a 15s+\n"
      "                                 real-time wait.\n"
      "  --stage-real-use-localization-source\n"
      "                                 TEST-ONLY: STAGE_REAL's current-pose\n"
      "                                 source becomes a dedicated listener on\n"
      "                                 --real-robots-config='s localization\n"
      "                                 endpoints instead of the OptiTrack\n"
      "                                 MocapManager -- lets a test stand in\n"
      "                                 ordinary mpc_robot_sim processes as\n"
      "                                 \"real hardware\" with no live bridge.\n"
      "                                 Default off (production: MocapManager).\n"
      "  --help                         Print this message and exit.\n";
}

// PART B save-path fix: --real-robots-config='s DEFAULT used to be the bare
// relative string "MARS/config/real_robots.json", resolved against this
// process's CWD -- correct when launched from the repo root (the documented
// production convention) but silently wrong from any other CWD, e.g. `cd
// build-release/MARS && ./mars_sim_viz` (the OptiTrack tab's "Save Mapping"
// button then tries to write to a nonexistent
// "build-release/MARS/MARS/config/" directory and fails). Resolves the
// SAME default relative to the executable's own location instead -- same
// convention as ExecutionManager::resolve_default_mpc_controller_path()/
// resolve_default_robot_sim_path() (SimVizCore.cpp) and MocapManager::
// resolve_bridge_path()/resolve_map_config_path() (MocapCore.cpp): built
// binaries live at "<build_dir>/MARS/mars_sim_viz", but (like
// mocap_map_config.json) real_robots.json is a SOURCE-tree file never
// copied into the build dir, so reaching it needs the same extra "/../"
// MocapManager::resolve_map_config_path() uses to climb out of the build
// dir entirely: "<build_dir>/MARS/../../MARS/config/real_robots.json"
// resolves to "<repo_root>/MARS/config/real_robots.json".
//
// Requires QCoreApplication to already exist (applicationDirPath() needs
// it) -- callers must invoke this AFTER constructing `app` in main(), not
// before.
std::string resolve_default_real_robots_config_path()
{
  const QString app_dir = QCoreApplication::applicationDirPath();
  const QString candidate = app_dir + "/../../MARS/config/real_robots.json";
  return QFileInfo(candidate).absoluteFilePath().toStdString();
}

// Finishes PART B's real-robots-config wiring: if the user never passed an
// explicit --real-robots-config=, replaces `real_robots_config_path` (so
// far just the CLI-parsing loop's placeholder) with
// resolve_default_real_robots_config_path()'s executable-relative default;
// an explicit path is left exactly as the user typed it (still resolved
// against CWD, same as every other CLI path override in this file). Then
// probes/loads it into `config` exactly as before (see this function's
// call sites in main() for why this had to move to AFTER `app`
// construction rather than staying inline in the argument-parsing block).
void finalize_real_robots_config(simviz::SimVizConfig &config, std::string real_robots_config_path,
                                  bool real_robots_config_explicit)
{
  if (!real_robots_config_explicit)
    real_robots_config_path = resolve_default_real_robots_config_path();

  std::ifstream probe(real_robots_config_path);
  if (probe.good())
  {
    probe.close();
    try
    {
      config.real_robot_endpoints = simviz::load_real_robots_config(real_robots_config_path);
      std::cout << "[mars_sim_viz] Loaded real-robot endpoints config '"
                << real_robots_config_path << "' (" << config.real_robot_endpoints.size()
                << " robot(s))" << std::endl;
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[mars_sim_viz] Warning: failed to load real-robots-config '"
                << real_robots_config_path << "': " << ex.what() << std::endl;
    }
  }
  else if (real_robots_config_explicit)
  {
    std::cerr << "[mars_sim_viz] Warning: --real-robots-config '" << real_robots_config_path
              << "' not found; every robot will use its fallback endpoint" << std::endl;
  }

  // DESIGN part C (GUI "Save Mapping" button): remember where the config
  // above came from (or would have come from) so a GUI-driven edit has
  // somewhere to write back to -- independent of whether the load above
  // actually succeeded (see SimVizConfig::real_robots_config_path's doc
  // comment).
  config.real_robots_config_path = real_robots_config_path;
}

} // namespace

int main(int argc, char **argv)
{
  simviz::SimVizConfig config;
  bool headless = false;
  std::string execute_path;
  bool exit_on_done = false;
  // PART B: only a placeholder when NOT overridden by --real-robots-config=
  // below -- finalize_real_robots_config() (called later, once `app`
  // exists) replaces it with an executable-relative default in that case,
  // never this literal (see that function's doc comment for why an
  // applicationDirPath()-relative default replaced the old bare CWD-relative
  // one). Left non-empty here only so an explicit --real-robots-config=
  // parsed below always wins over it, same as any other CLI override in
  // this file.
  std::string real_robots_config_path = "MARS/config/real_robots.json";
  bool real_robots_config_explicit = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--headless")
    {
      headless = true;
    }
    else if (starts_with(arg, "--execute="))
    {
      execute_path = arg.substr(std::string("--execute=").size());
    }
    else if (arg == "--exit-on-done")
    {
      exit_on_done = true;
    }
    else if (starts_with(arg, "--control-port="))
    {
      config.control_port = std::stoi(arg.substr(std::string("--control-port=").size()));
    }
    else if (starts_with(arg, "--handshake-port-start="))
    {
      config.handshake_port_start =
          std::stoi(arg.substr(std::string("--handshake-port-start=").size()));
    }
    else if (starts_with(arg, "--vesc-port-start="))
    {
      config.vesc_port_start = std::stoi(arg.substr(std::string("--vesc-port-start=").size()));
    }
    else if (starts_with(arg, "--loc-port-start="))
    {
      config.loc_port_start = std::stoi(arg.substr(std::string("--loc-port-start=").size()));
    }
    else if (starts_with(arg, "--mpc-controller-path="))
    {
      config.mpc_controller_path = arg.substr(std::string("--mpc-controller-path=").size());
    }
    else if (starts_with(arg, "--robot-sim-path="))
    {
      config.robot_sim_path = arg.substr(std::string("--robot-sim-path=").size());
    }
    else if (starts_with(arg, "--run-dir="))
    {
      config.run_dir = arg.substr(std::string("--run-dir=").size());
    }
    else if (starts_with(arg, "--noise-sigma-pct="))
    {
      // FEATURE C: applies to every spawn's --noise-sigma-pct= AND the
      // periodic control-socket/toolbar-slider publish path -- see
      // ExecutionManager's FEATURE C doc comments. Clamping to [0,0.25]
      // happens uniformly in ExecutionManager's constructor (via
      // clamp_noise_sigma_pct()), same as any later
      // set_noise_sigma_pct() call, so an out-of-range value here is
      // silently clamped rather than rejected.
      try
      {
        config.noise_sigma_pct =
            std::stod(arg.substr(std::string("--noise-sigma-pct=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --noise-sigma-pct value '" << arg
                  << "', defaulting to 0" << std::endl;
        config.noise_sigma_pct = 0.0;
      }
    }
    else if (starts_with(arg, "--steer-noise-sigma-pct="))
    {
      // PART (1) STEERING-NOISE SPLIT: independent STEER-channel counterpart
      // of --noise-sigma-pct= above -- same clamp-in-ExecutionManager's-
      // constructor behavior, same silently-default-to-0-on-parse-failure
      // behavior.
      try
      {
        config.steer_noise_sigma_pct =
            std::stod(arg.substr(std::string("--steer-noise-sigma-pct=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --steer-noise-sigma-pct value '" << arg
                  << "', defaulting to 0" << std::endl;
        config.steer_noise_sigma_pct = 0.0;
      }
    }
    else if (starts_with(arg, "--noise-seed="))
    {
      // FEATURE C (testability only): passthrough --noise-seed= for every
      // spawned mpc_robot_sim -- absent by default (nondeterministic seed
      // per sim, same as never passing --noise-seed to it directly).
      try
      {
        config.noise_seed = static_cast<std::uint64_t>(
            std::stoull(arg.substr(std::string("--noise-seed=").size())));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --noise-seed value '" << arg
                  << "', ignoring (spawns will use a nondeterministic seed)" << std::endl;
      }
    }
    else if (arg == "--deadband")
    {
      // MOTOR STALL: headless equivalent of the GUI's "Enable stall (min
      // speed)" checkbox -- independent of --stall-level= below, exactly
      // like the checkbox is independent of the level spinbox (setting the
      // level alone, without this flag, has no effect).
      config.stall_enabled = true;
    }
    else if (starts_with(arg, "--stall-level="))
    {
      // MOTOR STALL: headless equivalent of the GUI's "Stall level"
      // spinbox (m/s, [0,0.25], default 0.10 -- clamped in
      // ExecutionManager's constructor same as any later
      // set_stall_level() call). Inert unless --deadband is also passed.
      try
      {
        config.stall_level = std::stod(arg.substr(std::string("--stall-level=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --stall-level value '" << arg
                  << "', defaulting to 0.10" << std::endl;
        config.stall_level = 0.10;
      }
    }
    else if (arg == "--stage-first")
    {
      config.staging_enabled = true;
    }
    else if (starts_with(arg, "--staging-pos-tol="))
    {
      try
      {
        config.staging_pos_tol = std::stod(arg.substr(std::string("--staging-pos-tol=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --staging-pos-tol value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (starts_with(arg, "--staging-yaw-tol="))
    {
      try
      {
        config.staging_yaw_tol = std::stod(arg.substr(std::string("--staging-yaw-tol=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --staging-yaw-tol value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (starts_with(arg, "--staging-margin="))
    {
      try
      {
        config.staging_margin = std::stod(arg.substr(std::string("--staging-margin=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --staging-margin value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (starts_with(arg, "--staging-max-retries="))
    {
      try
      {
        config.staging_max_retries =
            std::stoi(arg.substr(std::string("--staging-max-retries=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --staging-max-retries value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (starts_with(arg, "--staging-localization-wait-s="))
    {
      try
      {
        config.staging_localization_wait_s =
            std::stod(arg.substr(std::string("--staging-localization-wait-s=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --staging-localization-wait-s value '"
                  << arg << "', keeping default" << std::endl;
      }
    }
    else if (starts_with(arg, "--staging-test-offsets="))
    {
      // TEST-ONLY -- see StagingCore.h's parse_staging_test_offsets() /
      // print_usage() above for the exact spec format.
      config.staging_test_offsets = arg.substr(std::string("--staging-test-offsets=").size());
    }
    else if (arg == "--real-mode")
    {
      // PART B: see SimVizConfig::real_mode's doc comment.
      config.real_mode = true;
    }
    else if (starts_with(arg, "--real-robots-config="))
    {
      real_robots_config_path = arg.substr(std::string("--real-robots-config=").size());
      real_robots_config_explicit = true;
    }
    else if (starts_with(arg, "--stage-real-pos-tol="))
    {
      try
      {
        config.stage_real_pos_tol =
            std::stod(arg.substr(std::string("--stage-real-pos-tol=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --stage-real-pos-tol value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (starts_with(arg, "--stage-real-yaw-tol="))
    {
      try
      {
        config.stage_real_yaw_tol =
            std::stod(arg.substr(std::string("--stage-real-yaw-tol=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --stage-real-yaw-tol value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (arg == "--stage-real-use-localization-source")
    {
      // TEST-ONLY -- see SimVizConfig::stage_real_use_localization_source's
      // doc comment.
      config.stage_real_use_localization_source = true;
    }
    else if (starts_with(arg, "--stage-real-leg-margin-s="))
    {
      // TEST-ONLY -- see SimVizConfig::stage_real_leg_timeout_margin_s's
      // doc comment.
      try
      {
        config.stage_real_leg_timeout_margin_s =
            std::stod(arg.substr(std::string("--stage-real-leg-margin-s=").size()));
      }
      catch (const std::exception &)
      {
        std::cerr << "[mars_sim_viz] Warning: invalid --stage-real-leg-margin-s value '" << arg
                  << "', keeping default" << std::endl;
      }
    }
    else if (arg == "--help" || arg == "-h")
    {
      print_usage();
      return 0;
    }
    else
    {
      std::cerr << "[mars_sim_viz] Warning: unrecognized argument '" << arg << "'" << std::endl;
    }
  }

  const bool have_signal_pipe = install_signal_handlers();

  if (headless)
  {
    QCoreApplication app(argc, argv);

    // PART B: best-effort load, AFTER the full argument-parsing loop above
    // (so an explicit --real-robots-config= is already known) AND after
    // `app` is constructed (an executable-relative default needs
    // QCoreApplication::applicationDirPath(), which requires it -- see
    // finalize_real_robots_config()'s doc comment). A missing DEFAULT path
    // is silent (real_mode may not even be requested this run); a missing
    // EXPLICITLY-requested path, or an unparsable file either way, is a
    // warning -- every robot simply falls back to its computed default
    // endpoint (see ExecutionManager::start_execution()'s real_mode
    // branch), never a fatal error.
    finalize_real_robots_config(config, real_robots_config_path, real_robots_config_explicit);

    simviz::SimVizManager manager(config);
    try
    {
      manager.start();
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[mars_sim_viz] Fatal: " << ex.what() << std::endl;
      return 1;
    }

    std::unique_ptr<QSocketNotifier> signal_notifier;
    if (have_signal_pipe)
    {
      signal_notifier = std::make_unique<QSocketNotifier>(g_signal_fd[1], QSocketNotifier::Read);
      QObject::connect(signal_notifier.get(), &QSocketNotifier::activated,
                        [&manager]() { handle_shutdown_signal(manager); });
    }

    std::cout << "[mars_sim_viz] (headless) Control socket bound on port " << config.control_port
              << ". Waiting for PING/EXECUTE..." << std::endl;

    // --execute: drive the exact same code path a control-socket
    // "EXECUTE <path>" request takes (SimVizManager::execute_scenario_file(),
    // which handle_command()'s EXECUTE branch itself now just forwards to).
    // Relative `execute_path` resolves against this process's CWD -- no
    // extra path manipulation here; ScenarioModel::load_from_file() opens it
    // via a plain std::ifstream, which already resolves relative paths
    // against getcwd() the normal way. On failure, exit promptly (before
    // ever entering the event loop) rather than idling with a control
    // socket nobody asked to keep alive.
    if (!execute_path.empty())
    {
      const std::string reply = manager.execute_scenario_file(execute_path);
      if (reply.rfind("ERR", 0) == 0)
      {
        std::cerr << "[mars_sim_viz] --execute '" << execute_path << "' failed: " << reply
                   << std::endl;
        return 1;
      }
      std::cout << "[mars_sim_viz] --execute '" << execute_path << "': " << reply << std::endl;
    }

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout,
                      [&]()
                      {
                        manager.tick();
                        if (!exit_on_done)
                          return;
                        // STAGING PHASE: exec_mgr_'s state transitions through
                        // Running/Done (and possibly Err, on a leg failure
                        // that fail_staging() then re-raises via
                        // fail_externally() once staging_ is no longer
                        // active) for EACH staging leg before the main run
                        // ever starts -- ignore Done/Err while a staging
                        // sequence is still in progress, or --exit-on-done
                        // would exit after the FIRST leg instead of the
                        // whole EXECUTE.
                        if (manager.is_staging_active())
                          return;
                        const simviz::RunState state = manager.execution_manager().state();
                        if (state == simviz::RunState::Done)
                        {
                          std::cout << "[mars_sim_viz] --exit-on-done: run DONE, exiting 0"
                                    << std::endl;
                          QCoreApplication::exit(0);
                        }
                        else if (state == simviz::RunState::Err)
                        {
                          std::cerr << "[mars_sim_viz] --exit-on-done: run ERR ("
                                    << manager.execution_manager().error_reason()
                                    << "), exiting 1" << std::endl;
                          QCoreApplication::exit(1);
                        }
                      });
    timer.start(50);

    return app.exec();
  }

  QApplication app(argc, argv);

  // PART B: see the matching call in the --headless branch above for why
  // this has to happen here (after `app` construction) rather than back in
  // the argument-parsing block.
  finalize_real_robots_config(config, real_robots_config_path, real_robots_config_explicit);

  simviz::SimVizWindow window(config);
  try
  {
    window.start();
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[mars_sim_viz] Fatal: " << ex.what() << std::endl;
    return 1;
  }

  std::unique_ptr<QSocketNotifier> signal_notifier;
  if (have_signal_pipe)
  {
    signal_notifier = std::make_unique<QSocketNotifier>(g_signal_fd[1], QSocketNotifier::Read);
    QObject::connect(signal_notifier.get(), &QSocketNotifier::activated,
                      [&window]() { handle_shutdown_signal(window.manager()); });
  }

  std::cout << "[mars_sim_viz] Control socket bound on port " << config.control_port
            << ". Waiting for PING/EXECUTE..." << std::endl;

  window.show();

  // --execute in GUI mode: exactly the File->Open code path
  // (SimVizWindow::open_scenario_file(), which loads + previews only --
  // see LOAD/START SPLIT in SimVizWindow.h's header doc comment). It no
  // longer auto-starts execution here; the operator presses the toolbar
  // Start button once ready (letting them connect/map mocap first). On
  // failure it already shows a QMessageBox and returns false; per spec we
  // stay open regardless (the user can retry via File->Open).
  if (!execute_path.empty())
  {
    window.open_scenario_file(QString::fromStdString(execute_path));
  }

  // --exit-on-done in GUI mode: documented (see print_usage()) as closing
  // the window once the run reaches DONE/ERR, rather than exiting the
  // process outright -- closing the last window still ends app.exec() via
  // Qt's default quitOnLastWindowClosed. Polled on its own timer (not
  // folded into SimVizWindow's internal 50ms tick timer) so this stays a
  // simviz_main.cpp-only concern; SimVizWindow itself knows nothing about
  // --exit-on-done.
  std::unique_ptr<QTimer> exit_on_done_timer;
  if (exit_on_done)
  {
    exit_on_done_timer = std::make_unique<QTimer>();
    QObject::connect(exit_on_done_timer.get(), &QTimer::timeout,
                      [&]()
                      {
                        // STAGING PHASE: same reasoning as the headless
                        // --exit-on-done handler above -- don't close the
                        // window on an intermediate staging leg's own
                        // Done/Err.
                        if (window.manager().is_staging_active())
                          return;
                        const simviz::RunState state =
                            window.manager().execution_manager().state();
                        if (state == simviz::RunState::Done)
                        {
                          std::cout << "[mars_sim_viz] --exit-on-done: run DONE, closing window"
                                    << std::endl;
                          window.close();
                        }
                        else if (state == simviz::RunState::Err)
                        {
                          std::cerr << "[mars_sim_viz] --exit-on-done: run ERR ("
                                    << window.manager().execution_manager().error_reason()
                                    << "), closing window" << std::endl;
                          window.close();
                        }
                      });
    exit_on_done_timer->start(200);
  }

  return app.exec();
}
