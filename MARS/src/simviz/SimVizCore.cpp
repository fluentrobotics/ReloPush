#include "SimVizCore.h"

#include "StagingCore.h"

#include <ReloPush/base64.h>

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcessEnvironment>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace simviz
{

namespace
{

constexpr int kControlRecvTimeoutMs = 0; // non-blocking poll
// Bounds ControlServer::reply()'s send() call. ZMQ's default SNDTIMEO is -1
// (block forever); a REP socket's send() rarely blocks for one small reply,
// but the failure mode if it ever did (e.g. a wedged/misbehaving peer that
// never drains its receive buffer) would hang SimVizManager::tick()'s whole
// loop indefinitely -- mirrors src/ReloPush/FinalSequenceHandoff.cpp, which
// sets both rcvtimeo and sndtimeo on its REP socket.
constexpr int kControlSendTimeoutMs = 2000;
constexpr int kHandshakeReadinessDelayMs = 500;
constexpr int kHandshakePerAttemptMs = 3000;
constexpr int kHandshakeAttempts = 2;
constexpr double kCompletionMarginS = 3.0;
// STAGE_REAL per-leg bounded timeout, per DESIGN: "traj duration + 15s".
// Deliberately SEPARATE from (and larger than) the generic
// ExecutionManager::tick() completion-forcing margin (kCompletionMarginS,
// above) -- STAGE_REAL legs run against real hardware, where "no
// convergence yet" more plausibly means "still genuinely in motion" than it
// does for a synthetic sim run, so a real leg gets Err'd out (not silently
// force-Done'd) once traj_duration + kStageRealLegTimeoutMarginS elapses
// without exec_mgr_ itself reaching Done/Err. See tick_stage_real().
constexpr double kStageRealLegTimeoutMarginS = 15.0;
constexpr int kTerminateGraceMs = 1500;
constexpr int kTerminatePollMs = 20;
// FEATURE 2B: rcvtimeo/sndtimeo for the fresh per-robot REQ socket
// pause_all()/resume_all() use to talk to mpc_controller's handshake REP
// socket (see SimVizHandoff.h's PING for the sibling one-shot-REQ pattern).
constexpr int kPauseResumeTimeoutMs = 2000;
// FEATURE C: min interval between idempotent noise-config republishes while
// a run is active (slow-joiner insurance for the sim-only
// "/<robot>/sim_config" topic -- see ExecutionManager::tick()).
constexpr double kNoisePublishIntervalS = 1.0;

// Mirrors mpc::NoiseModel::clamp_sigma_pct()'s range (MPC/include/
// mpc/SimCore.h) without adding a cross-module include -- MARS only needs
// the numeric range here, not mpc_robot_sim's RNG/sampling machinery.
// Non-finite input is treated as 0 (the safe "no noise" default).
double clamp_noise_sigma_pct(double sigma_pct)
{
  if (!std::isfinite(sigma_pct))
    return 0.0;
  return std::clamp(sigma_pct, 0.0, 0.25);
}

// MOTOR STALL: same [0,0.25] clamp range as clamp_noise_sigma_pct() above
// (matches the UI's "Stall level" spinbox range) but a distinct quantity
// (m/s, not a noise sigma fraction) -- kept as its own named function for
// clarity at call sites even though the clamp logic is identical.
double clamp_stall_level(double level_mps)
{
  if (!std::isfinite(level_mps))
    return 0.0;
  return std::clamp(level_mps, 0.0, 0.25);
}

std::string sanitize_for_path(const std::string &s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s)
  {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' || c == '.')
      out.push_back(c);
    else
      out.push_back('_');
  }
  if (out.empty())
    out = "run";
  return out;
}

// Sends `msg` on an already-connected REQ socket without waiting for a
// reply. Factored out of send_and_wait_reply so callers that need to
// broadcast to several sockets before collecting any replies (e.g. START,
// see run_handshake_thread's Phase 2/3 below) can do so -- each REQ socket
// is a separate connection, so sending on socket i does not block on socket
// i+1's reply.
bool send_only(zmq::socket_t &sock, const std::string &msg, std::string *error_out)
{
  const auto send_res = sock.send(zmq::buffer(msg), zmq::send_flags::none);
  if (!send_res)
  {
    if (error_out)
      *error_out = "send failed";
    return false;
  }
  return true;
}

// Polls recv() (NOT resending) up to kHandshakeAttempts times with
// kHandshakePerAttemptMs rcvtimeo each. Safe to retry this way (unlike
// SimVizHandoff's PING, which must use a fresh socket per attempt) because
// we only ever call recv() here -- never send() again -- so the REQ
// socket's send/recv alternation is never violated. See SimVizCore.h's
// ExecutionManager doc comment for why a resend-based retry would be unsafe
// against mpc_controller's fixed two-phase handshake.
bool wait_for_reply(zmq::socket_t &sock, const std::string &expect_prefix,
                     std::string *reply_out, std::string *error_out,
                     std::atomic<bool> *stop_flag)
{
  for (int attempt = 0; attempt < kHandshakeAttempts; ++attempt)
  {
    if (stop_flag && stop_flag->load())
    {
      if (error_out)
        *error_out = "aborted";
      return false;
    }
    sock.set(zmq::sockopt::rcvtimeo, kHandshakePerAttemptMs);
    zmq::message_t reply;
    const auto recv_res = sock.recv(reply, zmq::recv_flags::none);
    if (recv_res)
    {
      std::string s(static_cast<const char *>(reply.data()), reply.size());
      if (reply_out)
        *reply_out = s;
      if (s.rfind(expect_prefix, 0) == 0)
        return true;
      if (error_out)
        *error_out = "unexpected reply '" + s + "' (expected prefix '" + expect_prefix + "')";
      return false;
    }
    // timeout on this attempt; loop again (still permitted: we have not
    // called send() again, so the REQ FSM is unchanged).
  }
  if (error_out)
    *error_out = "timed out waiting for '" + expect_prefix + "'";
  return false;
}

// Sends `msg` on an already-connected REQ socket, then waits for a reply
// with the given prefix. Used where send-then-immediately-wait on the SAME
// socket is fine (trajectory upload, Phase 1 below) -- see send_only() /
// wait_for_reply() above for the broadcast-then-collect split START needs.
bool send_and_wait_reply(zmq::socket_t &sock, const std::string &msg,
                          const std::string &expect_prefix, std::string *reply_out,
                          std::string *error_out, std::atomic<bool> *stop_flag)
{
  if (!send_only(sock, msg, error_out))
    return false;
  return wait_for_reply(sock, expect_prefix, reply_out, error_out, stop_flag);
}

} // namespace

// ===========================================================================
// ScenarioModel
// ===========================================================================

ScenarioModel ScenarioModel::load_from_file(const std::string &path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open())
    throw std::runtime_error("ScenarioModel::load_from_file: cannot open '" + path + "'");

  std::ostringstream ss;
  ss << in.rdbuf();
  const std::string b64 = ss.str();
  if (b64.empty())
    throw std::runtime_error("ScenarioModel::load_from_file: '" + path + "' is empty");

  ExecutedScenario scn = deserialize_executed_scenario_b64(b64); // may throw

  ScenarioModel model;
  model.scenario_ = std::make_shared<ExecutedScenario>(std::move(scn));
  model.robot_trajectories_ =
      build_robot_trajectories(model.scenario_->timetable, model.robots_sorted_by_name());
  return model;
}

std::vector<EntityMeta *> ScenarioModel::robots_sorted_by_name() const
{
  std::vector<EntityMeta *> out;
  if (!scenario_)
    return out;
  for (const auto &[name, ent] : scenario_->entities)
  {
    if (ent && ent->type == EntityType::ROBOT)
      out.push_back(ent);
  }
  std::sort(out.begin(), out.end(),
            [](EntityMeta *a, EntityMeta *b) { return a->name < b->name; });
  return out;
}

std::vector<EntityMeta *> ScenarioModel::objects_sorted_by_name() const
{
  std::vector<EntityMeta *> out;
  if (!scenario_)
    return out;
  for (const auto &[name, ent] : scenario_->entities)
  {
    if (ent && ent->type == EntityType::OBJECT)
      out.push_back(ent);
  }
  std::sort(out.begin(), out.end(),
            [](EntityMeta *a, EntityMeta *b) { return a->name < b->name; });
  return out;
}

// ===========================================================================
// ControlServer
// ===========================================================================

ControlServer::~ControlServer() = default;

void ControlServer::bind(int port)
{
  ctx_ = std::make_unique<zmq::context_t>(1);
  sock_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::rep);
  sock_->set(zmq::sockopt::linger, 0);
  sock_->set(zmq::sockopt::rcvtimeo, kControlRecvTimeoutMs);
  sock_->set(zmq::sockopt::sndtimeo, kControlSendTimeoutMs);
  const std::string addr = "tcp://*:" + std::to_string(port);
  try
  {
    sock_->bind(addr);
  }
  catch (const std::exception &ex)
  {
    throw std::runtime_error("ControlServer::bind: failed to bind '" + addr +
                              "': " + ex.what());
  }
  bound_ = true;
}

std::optional<std::string> ControlServer::poll_once()
{
  if (!bound_)
    return std::nullopt;

  zmq::message_t msg;
  try
  {
    const auto res = sock_->recv(msg, zmq::recv_flags::dontwait);
    if (!res)
      return std::nullopt;
  }
  catch (const zmq::error_t &ex)
  {
    if (ex.num() == EAGAIN)
      return std::nullopt;
    return std::nullopt;
  }
  return std::string(static_cast<const char *>(msg.data()), msg.size());
}

void ControlServer::reply(const std::string &message)
{
  if (!bound_)
    return;
  try
  {
    const auto res = sock_->send(zmq::buffer(message), zmq::send_flags::none);
    if (!res)
    {
      std::cerr << "[mars_sim_viz] ControlServer::reply: send timed out after "
                << kControlSendTimeoutMs << "ms; reply '" << message << "' was dropped"
                << std::endl;
    }
  }
  catch (const zmq::error_t &ex)
  {
    std::cerr << "[mars_sim_viz] ControlServer::reply: send failed: " << ex.what() << std::endl;
  }
}

// ===========================================================================
// LocalizationListener
// ===========================================================================

LocalizationListener::~LocalizationListener()
{
  stop();
}

void LocalizationListener::start(const std::vector<std::pair<std::string, int>> &robots)
{
  std::vector<std::pair<std::string, std::string>> endpoints;
  endpoints.reserve(robots.size());
  for (const auto &[name, port] : robots)
    endpoints.emplace_back(name, "tcp://127.0.0.1:" + std::to_string(port));
  start_with_endpoints(endpoints);
}

void LocalizationListener::start_with_endpoints(
    const std::vector<std::pair<std::string, std::string>> &robot_endpoints)
{
  stop();
  // Stamped even if `robot_endpoints` is empty below -- has_fresh_pose()
  // callers (STAGING PHASE "hardware mode" presence checks) need a valid
  // basis as soon as a run's listener has (attempted to) start, not only
  // once it successfully has at least one robot to track.
  start_time_ = std::chrono::steady_clock::now();
  start_time_valid_ = true;
  if (robot_endpoints.empty())
    return;
  stop_requested_.store(false);
  thread_ = std::thread(&LocalizationListener::thread_main, this, robot_endpoints);
}

void LocalizationListener::stop()
{
  stop_requested_.store(true);
  if (thread_.joinable())
    thread_.join();
  // ISSUE 1 fix: deliberately NOT clearing poses_ here -- see this method's
  // doc comment in SimVizCore.h. Callers that need a clean slate (a
  // genuinely new run) call reset() explicitly.
}

void LocalizationListener::reset()
{
  std::lock_guard<std::mutex> lk(poses_mutex_);
  poses_.clear();
  telemetry_.clear();
}

std::optional<LiveRobotPose> LocalizationListener::latest_pose(const std::string &name) const
{
  std::lock_guard<std::mutex> lk(poses_mutex_);
  auto it = poses_.find(name);
  if (it == poses_.end())
    return std::nullopt;
  return it->second;
}

std::optional<LiveRobotTelemetry> LocalizationListener::latest_telemetry(const std::string &name) const
{
  std::lock_guard<std::mutex> lk(poses_mutex_);
  auto it = telemetry_.find(name);
  if (it == telemetry_.end())
    return std::nullopt;
  return it->second;
}

bool LocalizationListener::has_fresh_pose(const std::string &robot_name, double max_age_s) const
{
  if (!start_time_valid_)
    return false;
  std::lock_guard<std::mutex> lk(poses_mutex_);
  auto it = poses_.find(robot_name);
  if (it == poses_.end() || !it->second.has_pose)
    return false;
  const double now_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  const double age_s = now_s - it->second.last_update_steady_s;
  return age_s >= 0.0 && age_s <= max_age_s;
}

void LocalizationListener::thread_main(
    std::vector<std::pair<std::string, std::string>> robot_endpoints)
{
  try
  {
    zmq::context_t ctx(1);
    std::vector<zmq::socket_t> socks;
    std::vector<std::string> names;
    // Parallel to `names` -- the exact telemetry topic string for that
    // robot/socket index, so the recv loop below can tell the two topics
    // apart by comparing the received topic frame against this (anything
    // else received on the same socket is the localization topic, the only
    // other one subscribed below).
    std::vector<std::string> telemetry_topics;
    socks.reserve(robot_endpoints.size());
    names.reserve(robot_endpoints.size());
    telemetry_topics.reserve(robot_endpoints.size());

    for (const auto &[name, endpoint] : robot_endpoints)
    {
      zmq::socket_t sock(ctx, zmq::socket_type::sub);
      sock.set(zmq::sockopt::linger, 0);
      sock.set(zmq::sockopt::rcvtimeo, 200);
      try
      {
        sock.connect(endpoint);
        const std::string loc_topic = "/" + name + "/localization";
        sock.set(zmq::sockopt::subscribe, loc_topic);
        // Telemetry rides the SAME bound PUB socket as localization (see
        // MPC/src/robot_sim.cpp / SimCore.h's TelemetrySample doc
        // comment), under its own topic -- subscribing to it here, on this
        // already-connected socket, is the least-churn way to track both
        // (no second socket/thread needed).
        const std::string telem_topic = "/" + name + "/telemetry";
        sock.set(zmq::sockopt::subscribe, telem_topic);
        telemetry_topics.push_back(telem_topic);
      }
      catch (const std::exception &)
      {
        continue;
      }
      socks.push_back(std::move(sock));
      names.push_back(name);
    }

    const auto t_start = std::chrono::steady_clock::now();

    while (!stop_requested_.load())
    {
      for (size_t i = 0; i < socks.size(); ++i)
      {
        // LAG-DIAGNOSIS BUGFIX (three-clock lag-diagnosis task): DRAIN every
        // currently-queued message on this socket, not just one. The
        // original code did a single dontwait recv() per socket per ~20ms
        // outer-loop cycle (one iteration processes ALL robots' sockets
        // once, then sleeps 20ms) -- but localization AND telemetry are
        // BOTH published at --loc-rate-hz (default 30Hz, robot_sim.cpp),
        // i.e. ~2 messages/~33ms per robot, while this loop could only ever
        // drain ~1 message/~20ms per robot. ZMQ SUB sockets queue backlog
        // rather than dropping old messages for new ones (no ZMQ_CONFLATE
        // here), so whenever combined arrival outpaced the single-recv
        // drain rate, latest_pose()/latest_telemetry() would silently
        // return an ever-more-stale cached sample -- confirmed live (this
        // task's report) on a real 3-robot run: a robot's cached pose was
        // ~26+ seconds behind its true simulated position by ~90s into the
        // run and still falling further behind, while the robot's TRUE
        // position (per its own CSV log) tracked the reference to
        // centimeter accuracy the whole time. This exact mechanism -- the
        // GUI's rendering of a robot's OWN live position falling further
        // and further behind over a long run, even though the reference
        // overlay and the robot's true position are both fine -- is a
        // strong candidate for what this task's user bug report actually
        // saw. Fix: loop the same body until a dontwait recv() finds
        // nothing left, so the cache always reflects the NEWEST available
        // sample every cycle regardless of run length or robot count.
        while (true)
        {
        zmq::message_t topic_msg;
        auto res = socks[i].recv(topic_msg, zmq::recv_flags::dontwait);
        if (!res)
          break; // nothing queued right now -- do NOT busy-spin; move to the next socket.
        const std::string topic_str(static_cast<const char *>(topic_msg.data()), topic_msg.size());
        std::string payload;
        if (topic_msg.more())
        {
          zmq::message_t payload_msg;
          if (socks[i].recv(payload_msg, zmq::recv_flags::none))
            payload.assign(static_cast<const char *>(payload_msg.data()), payload_msg.size());
        }
        if (payload.empty())
          continue;

        // Plain-JSON numeric-field extractor, shared by both topics below
        // -- see robot_sim.cpp's encode_localization_payload()/
        // mpc::encode_telemetry_payload(). Minimal hand-rolled parse to
        // avoid pulling nlohmann::json into mars_sim_viz just for this
        // (matches this project's existing convention here).
        auto extract = [&](const char *key) -> std::optional<double> {
          const std::string needle = std::string("\"") + key + "\"";
          auto pos = payload.find(needle);
          if (pos == std::string::npos)
            return std::nullopt;
          pos = payload.find(':', pos);
          if (pos == std::string::npos)
            return std::nullopt;
          try
          {
            return std::stod(payload.substr(pos + 1));
          }
          catch (...)
          {
            return std::nullopt;
          }
        };

        const double now_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

        if (topic_str == telemetry_topics[i])
        {
          // {"t":..,"v":..,"v_cmd":..,"steering":..,"accel":..,"moving":0|1,
          // "watchdog":0|1} -- mpc::encode_telemetry_payload()'s exact wire
          // format (SimCore.h). moving/watchdog are JSON integers 0/1, so
          // extract() (which parses via std::stod) handles them fine too.
          auto t = extract("t");
          auto v = extract("v");
          auto v_cmd = extract("v_cmd");
          auto steering = extract("steering");
          auto accel = extract("accel");
          auto moving = extract("moving");
          auto watchdog = extract("watchdog");
          if (!t || !v || !v_cmd || !steering || !accel || !moving || !watchdog)
            continue;

          LiveRobotTelemetry live;
          live.t = *t;
          live.v = *v;
          live.v_cmd = *v_cmd;
          live.steering = *steering;
          live.accel = *accel;
          live.moving = (*moving != 0.0);
          live.watchdog = (*watchdog != 0.0);
          live.has_telemetry = true;
          live.last_update_steady_s = now_s;

          std::lock_guard<std::mutex> lk(poses_mutex_);
          telemetry_[names[i]] = live;
          continue;
        }

        // Otherwise: the localization topic (the only other one subscribed
        // above) -- {"x":..,"y":..,"yaw":..}.
        auto x = extract("x");
        auto y = extract("y");
        auto yaw = extract("yaw");
        if (!x || !y || !yaw)
          continue;

        LiveRobotPose live;
        live.pose = Pose(*x, *y, *yaw);
        live.has_pose = true;
        live.last_update_steady_s = now_s;

        std::lock_guard<std::mutex> lk(poses_mutex_);
        poses_[names[i]] = live;
        } // while (true) -- drain loop for this socket, see BUGFIX comment above.
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  catch (const std::exception &)
  {
    // Background diagnostics thread -- swallow errors, just stop updating.
  }
}

// ===========================================================================
// ExecutionManager
// ===========================================================================

const char *run_state_name(RunState state)
{
  switch (state)
  {
  case RunState::Idle:
    return "Idle";
  case RunState::Running:
    return "Running";
  case RunState::Done:
    return "Done";
  case RunState::Err:
    return "Err";
  }
  return "Unknown";
}

// PART B: see this function's doc comment in SimVizCore.h.
std::unordered_map<std::string, RealRobotEndpoints>
load_real_robots_config(const std::string &path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open())
    throw std::runtime_error("load_real_robots_config: cannot open '" + path + "'");

  nlohmann::json j;
  in >> j; // may throw nlohmann::json::parse_error

  std::unordered_map<std::string, RealRobotEndpoints> out;
  if (!j.contains("robots") || !j["robots"].is_object())
    return out;

  for (auto it = j["robots"].begin(); it != j["robots"].end(); ++it)
  {
    RealRobotEndpoints ep;
    if (it.value().is_object())
    {
      ep.vesc_endpoint = it.value().value("vesc_endpoint", std::string());
      ep.localization_endpoint = it.value().value("localization_endpoint", std::string());
    }
    out[it.key()] = ep;
  }
  return out;
}

bool save_real_robots_config(const std::unordered_map<std::string, RealRobotEndpoints> &endpoints,
                              const std::string &path, std::string *err)
{
  if (path.empty())
  {
    if (err)
      *err = "save_real_robots_config: empty path";
    return false;
  }

  // Preserve "_doc" from whatever is currently on disk (best-effort -- a
  // missing/unparsable existing file just falls back to a canned default),
  // same "don't clobber unrelated fields" spirit as MocapCore.h's
  // set_alias_in_mapping()/save_mapping_json() round trip.
  nlohmann::json doc =
      "PART B (REAL-ROBOT MODE): per-robot mpc_controller endpoint overrides for mars_sim_viz "
      "--real-mode. See MARS/src/simviz/SimVizCore.h's load_real_robots_config()/"
      "RealRobotEndpoints doc comments for the full contract. Written by the OptiTrack tab's "
      "'Save Mapping' button.";
  {
    std::ifstream existing(path, std::ios::binary);
    if (existing.is_open())
    {
      try
      {
        nlohmann::json existing_json;
        existing >> existing_json;
        if (existing_json.contains("_doc"))
          doc = existing_json["_doc"];
      }
      catch (const std::exception &)
      {
        // Unparsable existing file -- fall back to the canned _doc above.
      }
    }
  }

  nlohmann::json out;
  out["_doc"] = doc;
  out["robots"] = nlohmann::json::object();
  for (const auto &[name, ep] : endpoints)
  {
    out["robots"][name] = {{"vesc_endpoint", ep.vesc_endpoint},
                            {"localization_endpoint", ep.localization_endpoint}};
  }

  const QFileInfo info(QString::fromStdString(path));
  const std::string abs_path = info.absoluteFilePath().toStdString();

  // Create the parent directory if it's missing -- e.g. a relative default
  // like "MARS/config/real_robots.json" resolved against an unexpected CWD
  // (mars_sim_viz launched from build-release/MARS) used to fail here with
  // no indication of WHY; MocapCore.h's save_mapping_json() applies the
  // same fix for the mapping-config write path, same rationale.
  const QDir parent_dir = info.dir();
  if (!parent_dir.exists() && !parent_dir.mkpath("."))
  {
    if (err)
      *err = "save_real_robots_config: cannot create directory '" +
             parent_dir.absolutePath().toStdString() + "' for '" + abs_path + "'";
    return false;
  }

  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f.is_open())
  {
    if (err)
      *err = "save_real_robots_config: cannot open '" + abs_path +
             "' for writing: " + std::strerror(errno);
    return false;
  }
  f << out.dump(2);
  if (!f.good())
  {
    if (err)
      *err = "save_real_robots_config: write failure on '" + abs_path +
             "': " + std::strerror(errno);
    return false;
  }
  return true;
}

ExecutionManager::ExecutionManager(SimVizConfig config, QObject *parent)
    : QObject(parent), config_(std::move(config))
{
  // FEATURE C: a run started before anyone ever touches the noise sliders
  // still honors SimVizConfig::noise_sigma_pct / steer_noise_sigma_pct (e.g.
  // mars_sim_viz --noise-sigma-pct=N / --steer-noise-sigma-pct=N in headless
  // mode) from t=0.
  noise_sigma_pct_ = clamp_noise_sigma_pct(config_.noise_sigma_pct);
  steer_noise_sigma_pct_ = clamp_noise_sigma_pct(config_.steer_noise_sigma_pct);
  // MOTOR STALL: same "a run started before anyone ever touches the UI
  // still honors the CLI-supplied initial config" rule as the noise sigmas
  // above (mars_sim_viz --deadband --stall-level=N in headless mode).
  stall_enabled_ = config_.stall_enabled;
  stall_level_ = clamp_stall_level(config_.stall_level);
}

ExecutionManager::~ExecutionManager()
{
  abort();
}

std::string ExecutionManager::resolve_mpc_controller_path() const
{
  if (!config_.mpc_controller_path.empty())
    return config_.mpc_controller_path;
  const QString app_dir = QCoreApplication::applicationDirPath();
  const QString candidate = app_dir + "/../MPC/mpc_controller";
  return QFileInfo(candidate).absoluteFilePath().toStdString();
}

std::string ExecutionManager::resolve_robot_sim_path() const
{
  if (!config_.robot_sim_path.empty())
    return config_.robot_sim_path;
  const QString app_dir = QCoreApplication::applicationDirPath();
  const QString candidate = app_dir + "/../MPC/mpc_robot_sim";
  return QFileInfo(candidate).absoluteFilePath().toStdString();
}

namespace
{
// Spawns `binary_path args...` wrapped in "env -i HOME=.. USER=.. PATH=..
// QT_QPA_PLATFORM=offscreen" -- matching ui/LauncherWindow.cpp's QProcess
// pattern (see ui/README.md) and this task's "run binaries wrapped" rule.
QProcess *spawn_wrapped_process(QObject *parent, const std::string &binary_path,
                                 const QStringList &args)
{
  auto *proc = new QProcess(parent);
  proc->setProcessChannelMode(QProcess::ForwardedChannels);

  const QString home = QProcessEnvironment::systemEnvironment().value("HOME");
  const QString user = QProcessEnvironment::systemEnvironment().value("USER");

  QStringList env_args;
  env_args << "-i" << ("HOME=" + home) << ("USER=" + user)
           << "PATH=/usr/local/bin:/usr/bin:/bin"
           << "QT_QPA_PLATFORM=offscreen" << QString::fromStdString(binary_path);
  env_args << args;

  proc->setProgram("env");
  proc->setArguments(env_args);
  proc->start();
  return proc;
}
} // namespace

bool ExecutionManager::start_execution(
    std::shared_ptr<ScenarioModel> scenario, const std::vector<std::string> &robot_subset,
    const std::unordered_map<std::string, ReloPush::trajectory> &trajectory_overrides,
    const std::string &label_suffix)
{
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ == RunState::Running)
      return false;
  }

  // A previous Done/Err run's children are already reaped by finalize_run();
  // this is just defensive cleanup in case of a direct re-entry.
  handshake_stop_requested_.store(true);
  terminate_children();
  if (handshake_thread_.joinable())
    handshake_thread_.join();
  localization_.stop();
  handshake_stop_requested_.store(false);

  // ISSUE 1 fix: this is the ONE place a genuinely new run begins (EXECUTE /
  // File->Open / RESTART's re-execute all funnel through start_execution())
  // -- clear any previous run's last-known poses now, so this fresh
  // scenario's initial_pose is the correct pre-localization fallback rather
  // than a stale pose left over from whatever ran before. See
  // LocalizationListener::reset()'s doc comment.
  localization_.reset();

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    current_scenario_ = scenario;
  }

  auto robots = scenario->robots_sorted_by_name();
  // STAGING PHASE: `robot_subset` (non-empty) restricts this run to just
  // those robots -- a staging leg's own single-robot mini-execution, or the
  // main run's present-robot subset once some robots were found MISSING.
  // Order is preserved from robots_sorted_by_name() (still name-sorted,
  // since it's a filter, not a re-sort) so port-index assignment below
  // stays deterministic for a given subset.
  if (!robot_subset.empty())
  {
    std::vector<EntityMeta *> filtered;
    filtered.reserve(robots.size());
    for (EntityMeta *robot : robots)
    {
      if (std::find(robot_subset.begin(), robot_subset.end(), robot->name) != robot_subset.end())
        filtered.push_back(robot);
    }
    robots = std::move(filtered);
  }
  if (robots.empty())
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    state_ = RunState::Err;
    label_ = scenario->label();
    error_reason_ = "scenario has no robot entities";
    return true; // EXECUTE was accepted; the run immediately errored out.
  }

  const std::string run_label = sanitize_for_path(scenario->label() + label_suffix);
  const QString run_dir_q =
      QString::fromStdString(config_.run_dir) + "/" + QString::fromStdString(run_label) + "_" +
      QString::number(static_cast<qlonglong>(QCoreApplication::applicationPid()));
  QDir().mkpath(run_dir_q);

  const std::string controller_exe = resolve_mpc_controller_path();
  const std::string sim_exe = resolve_robot_sim_path();

  std::vector<std::pair<std::string, std::string>> loc_endpoints;
  std::vector<std::string> robot_names;
  std::vector<int> handshake_ports;
  std::vector<ReloPush::trajectory> trajectories;

  children_.clear();
  children_.reserve(robots.size());

  // FEATURE C: a fresh context + PUB-socket set for THIS run's noise-config
  // channel (see NoisePubTarget's doc comment in SimVizCore.h); the
  // previous run's, if any, were already torn down above by
  // terminate_children(). Read the spawn-time noise sigma once, under lock,
  // so every robot in this run starts from the same value even if a slider
  // change races this loop.
  noise_pub_targets_.clear();
  noise_pub_ctx_ = std::make_unique<zmq::context_t>(1);
  noise_last_publish_valid_ = false;
  double spawn_noise_sigma_pct;
  double spawn_steer_noise_sigma_pct;
  bool spawn_stall_enabled;
  double spawn_stall_level;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    spawn_noise_sigma_pct = noise_sigma_pct_;
    spawn_steer_noise_sigma_pct = steer_noise_sigma_pct_;
    spawn_stall_enabled = stall_enabled_;
    spawn_stall_level = stall_level_;
  }
  // ANTI-RE-STALL HYSTERESIS: sustain is a FIXED fraction (0.7, kinetic <
  // static friction -- ratio configurable later if a real robot's measured
  // values call for it) of the stall LEVEL, while min_moving_speed stays
  // exactly the level itself -- min_moving_speed/min_sustain_speed remain
  // the ONE user-facing "stall level" slider/flag on both the sim and
  // controller sides (same as before this change), just no longer equal to
  // each other. A zero hysteresis gap (sustain==moving, the pre-existing
  // behavior) is physically wrong: real breakaway (static) friction is
  // always >= sustained (kinetic) friction, so a robot that can already
  // sustain motion at its OWN breakaway speed should not re-stall the
  // instant actuation noise nudges it a hair below that same threshold.
  const double spawn_stall_sustain = 0.7 * spawn_stall_level;

  // MOTOR STALL: when enabled, every robot in this run shares one
  // mpc_controller --robot-spec override (same stall_level for
  // min_moving_speed/min_sustain_speed on every robot, "for now" -- see
  // SimVizConfig::stall_level's doc comment), so it is written ONCE here
  // rather than per-robot inside the loop below. Only the two deadband
  // fields are set -- load_spec_from_json() only overrides keys actually
  // present in the file (see config/robot_spec.json's "_doc"), so this
  // leaves every other geometry/limit/MPC-weight field at its compiled
  // default. Left empty (never written, never passed) when disabled --
  // controllers then get no --robot-spec at all, i.e. compiled defaults,
  // exactly as if this feature did not exist.
  std::string stall_robot_spec_path;
  if (spawn_stall_enabled)
  {
    stall_robot_spec_path = (run_dir_q + "/stall_robot_spec_override.json").toStdString();
    std::ostringstream spec_json;
    spec_json << "{\"robot\":{\"min_moving_speed\":" << spawn_stall_level
              << ",\"min_sustain_speed\":" << spawn_stall_sustain << "}}";
    std::ofstream spec_out(stall_robot_spec_path);
    if (spec_out)
    {
      spec_out << spec_json.str();
    }
    else
    {
      std::cerr << "[mars_sim_viz] Warning: MOTOR STALL failed to write robot-spec override '"
                 << stall_robot_spec_path
                 << "' -- spawned mpc_controller(s) will use compiled defaults instead."
                 << std::endl;
      stall_robot_spec_path.clear();
    }
  }

  // Looked up BY NAME rather than by index below: `robots` may now be a
  // filtered subset (robot_subset above), so it no longer necessarily lines
  // up 1:1 with scenario->robot_trajectories() (built from the FULL,
  // unfiltered robots_sorted_by_name() list). `trajectory_overrides` (also
  // by name) takes priority when a robot's name has an entry there --
  // STAGING PHASE's per-leg trajectory instead of the scenario's own.
  std::unordered_map<std::string, ReloPush::trajectory> traj_by_name;
  for (const auto &pr : scenario->robot_trajectories())
  {
    if (pr.first)
      traj_by_name[pr.first->name] = pr.second;
  }

  for (size_t i = 0; i < robots.size(); ++i)
  {
    EntityMeta *robot = robots[i];
    const int handshake_port = config_.handshake_port_start + static_cast<int>(i);
    const int vesc_port = config_.vesc_port_start + static_cast<int>(i);
    int loc_port = config_.loc_port_start + static_cast<int>(i);

    ReloPush::trajectory traj;
    if (auto override_it = trajectory_overrides.find(robot->name);
        override_it != trajectory_overrides.end())
      traj = override_it->second;
    else if (auto it = traj_by_name.find(robot->name); it != traj_by_name.end())
      traj = it->second;

    double x0 = robot->initial_pose.x, y0 = robot->initial_pose.y,
           yaw0 = robot->initial_pose.yaw;
    if (traj.trajectory_points && !traj.trajectory_points->empty())
    {
      const auto &wp0 = traj.trajectory_points->front();
      x0 = wp0.x;
      y0 = wp0.y;
      yaw0 = wp0.yaw;
    }

    const std::string csv_path =
        (run_dir_q + "/" + QString::fromStdString(robot->name) + ".csv").toStdString();

    // UNMAPPED ROBOTS ARE SIM-ONLY (mixed real/sim fleet): PER-ROBOT real-
    // vs-sim decision, not a blanket config_.real_mode. A robot is wired as
    // REAL iff EITHER (a) real_mode is on AND (no MocapManager source was
    // ever set -- see set_mocap_source()'s doc comment, preserving every
    // direct-ExecutionManager caller's pre-existing all-real behavior -- OR
    // this robot IS mapped), OR (b) no explicit --real-mode was requested at
    // all but the mocap source is live (state() == Connected) AND this robot
    // is mapped -- MIXED-FLEET AUTO-LINK: a GUI Start press with mocap
    // Connected + a mapping should not require a separate --real-mode CLI
    // flag from the operator (this is what the bug report's "GUI likely
    // never sets real_mode" gap was -- config_.real_mode alone gated 100% of
    // this decision, so a GUI-loaded run with mocap connected and a robot
    // mapped still simulated EVERY robot, wrong-robot canvas pose and all).
    // An unmapped robot always falls through to exactly the sim-mode branch
    // below in either case.
    const bool has_mocap_source = mocap_for_real_mode_ != nullptr;
    const bool mocap_is_connected = has_mocap_source && mocap_for_real_mode_->state() ==
                                                              MocapState::Connected;
    const bool robot_is_mapped_flag =
        has_mocap_source && robot_is_mapped(mocap_for_real_mode_->mapping(), robot->name);
    const bool robot_is_real = config_.real_mode
                                    ? (!has_mocap_source || robot_is_mapped_flag)
                                    : (mocap_is_connected && robot_is_mapped_flag);

    // PORT COLLISION FIX (mixed-fleet): config_.loc_port_start (sim robots'
    // per-index localization PUB port, default 3260) and config_.mocap.
    // loc_port_start (the OptiTrack bridge's single auto-discovery PUB port,
    // ALSO default 3260) share the same default value. In a mixed run with
    // the bridge live, whichever simulated robot's index lands on exactly
    // that port (index 0 at the defaults, i.e. the alphabetically-first
    // robot) has its own mpc_robot_sim fail to bind ("Address already in
    // use") and exit immediately (see robot_sim.cpp's loc_pub.bind() catch
    // block, which returns 1 -- there is no retry), while its controller's
    // --localization-endpoint (same port) silently ends up wired to the
    // BRIDGE's socket instead, subscribed to a topic ("/<this robot's
    // name>/localization") the bridge never publishes (it only publishes
    // under each mapped body's OWN alias) -- net effect: that robot never
    // receives any pose at all and the canvas shows it stuck at whatever
    // placeholder pose preceded the run, i.e. "somewhere not related" to
    // its actual scenario initial pose. Only SIMULATED robots spawn
    // mpc_robot_sim (a REAL/mocap-linked robot never binds a loc port of
    // its own -- it uses the bridge's port on purpose, see loc_endpoint_str
    // below), and only when the bridge is actually live does it actually
    // hold config_.mocap.loc_port_start as an OS-level port -- so this only
    // ever perturbs a simulated robot's port when it would otherwise
    // collide with a running bridge. The fallback offset (+1000) is chosen
    // to sit far outside any plausible per-index fleet range.
    //
    // Gated on the bridge PROCESS being alive (Connecting OR Connected),
    // NOT narrowly on Connected: the bridge's ZMQ PUB socket is bound as
    // soon as its process starts, well before MocapManager's own state
    // reaches Connected (which additionally requires a live body frame) --
    // the OS-level port conflict exists during that whole Connecting
    // window too.
    const bool mocap_bridge_running =
        has_mocap_source && (mocap_for_real_mode_->state() == MocapState::Connecting ||
                              mocap_for_real_mode_->state() == MocapState::Connected);
    if (!robot_is_real && mocap_bridge_running && loc_port == config_.mocap.loc_port_start)
    {
      loc_port = config_.mocap.loc_port_start + 1000 + static_cast<int>(i);
    }

    // PART B: REAL-ROBOT MODE. Resolve this robot's controller endpoints.
    // Sim mode (default, and every UNMAPPED robot in a mixed real_mode run)
    // uses the fixed vesc_port_start+i/loc_port_start+i ports this manager
    // itself spawns mpc_robot_sim to bind, exactly as before this feature
    // existed. Real mode looks up config_.real_robot_endpoints by robot
    // name, falling back to the SAME vesc_port formula (so an unconfigured
    // robot's controller still has a well-defined vesc target) and to the
    // single OptiTrack bridge auto-discovery port (config_.mocap.
    // loc_port_start) for localization -- see MocapCore.h's AUTO-DISCOVERY
    // doc comment: every body, including this one, publishes on that ONE
    // port under its own "/<name>/localization" topic.
    std::string vesc_endpoint = "tcp://127.0.0.1:" + std::to_string(vesc_port);
    std::string loc_endpoint_str = "tcp://127.0.0.1:" + std::to_string(loc_port);
    if (robot_is_real)
    {
      loc_endpoint_str = "tcp://127.0.0.1:" + std::to_string(config_.mocap.loc_port_start);
      auto override_it = config_.real_robot_endpoints.find(robot->name);
      if (override_it != config_.real_robot_endpoints.end())
      {
        if (!override_it->second.vesc_endpoint.empty())
          vesc_endpoint = override_it->second.vesc_endpoint;
        if (!override_it->second.localization_endpoint.empty())
          loc_endpoint_str = override_it->second.localization_endpoint;
      }
    }

    // PART B: a REAL robot NEVER spawns mpc_robot_sim (the whole point --
    // "real hardware" stands in its place) and skips the FEATURE C
    // noise-config PUB connect below (there is no sim on the other end to
    // receive "/<robot>/sim_config" -- real hardware doesn't understand
    // it, and stall/noise are sim-only plant-model knobs). An UNMAPPED
    // robot in a mixed real_mode run takes this exact branch too, same as
    // plain sim mode.
    QProcess *sim_proc = nullptr;
    if (!robot_is_real)
    {
      QStringList sim_args;
      sim_args << "--robot-name" << QString::fromStdString(robot->name) << "--cmd-endpoint"
               << ("tcp://*:" + QString::number(vesc_port)) << "--loc-endpoint"
               << ("tcp://*:" + QString::number(loc_port)) << "--x" << QString::number(x0, 'f', 6)
               << "--y" << QString::number(y0, 'f', 6) << "--yaw" << QString::number(yaw0, 'f', 6)
               << "--log-csv" << QString::fromStdString(csv_path);
      // FEATURE C: fresh runs start with the sliders' (or
      // --noise-sigma-pct=/--steer-noise-sigma-pct=) values from t=0.
      // Single-token "--flag=value" form -- mpc_robot_sim's CLI parser
      // (robot_sim.cpp's parse_args) only recognizes these noise flags in
      // that form, not as separate "--flag" "value" tokens.
      sim_args << ("--noise-sigma-pct=" + QString::number(spawn_noise_sigma_pct, 'f', 6));
      // PART (1) STEERING-NOISE SPLIT: independent channel, same wire flag
      // convention as above.
      sim_args << ("--steer-noise-sigma-pct=" +
                    QString::number(spawn_steer_noise_sigma_pct, 'f', 6));
      if (config_.noise_seed.has_value())
        sim_args << ("--noise-seed=" +
                      QString::number(static_cast<qulonglong>(*config_.noise_seed)));
      // MOTOR STALL: opt-in --deadband + explicit thresholds. An explicit
      // --min-moving-speed=/--min-sustain-speed= override alone would already
      // imply enabling FEATURE B on the sim side (see robot_sim.cpp's
      // parse_args), but passing --deadband explicitly too keeps this call
      // self-documenting.
      if (spawn_stall_enabled)
      {
        sim_args << "--deadband";
        sim_args << ("--min-moving-speed=" + QString::number(spawn_stall_level, 'f', 6));
        sim_args << ("--min-sustain-speed=" + QString::number(spawn_stall_sustain, 'f', 6));
      }
      sim_proc = spawn_wrapped_process(this, sim_exe, sim_args);

      // FEATURE C: PUB-connect (not bind) to this robot's own mpc_robot_sim
      // --cmd-endpoint -- the exact same endpoint mpc_controller's ackermann
      // PUB connects to, i.e. a second PUB peer of the sim's one bound SUB
      // socket. Used later by publish_sim_config() to send
      // {"noise_sigma_pct": v, ..., "deadband": 0|1, ...} on the sim-only
      // "/<robot>/sim_config" topic (MOTOR STALL rides this same channel).
      auto sock = std::make_unique<zmq::socket_t>(*noise_pub_ctx_, zmq::socket_type::pub);
      sock->set(zmq::sockopt::linger, 0);
      try
      {
        sock->connect("tcp://127.0.0.1:" + std::to_string(vesc_port));
      }
      catch (const std::exception &ex)
      {
        std::cerr << "[mars_sim_viz] Warning: FEATURE C noise-config PUB connect failed for '"
                   << robot->name << "': " << ex.what() << std::endl;
      }
      noise_pub_targets_.push_back(NoisePubTarget{robot->name, std::move(sock)});
    }

    QStringList ctl_args;
    ctl_args << "--robot" << QString::fromStdString(robot->name) << "--port"
             << QString::number(handshake_port) << "--vesc-endpoint"
             << QString::fromStdString(vesc_endpoint) << "--localization-endpoint"
             << QString::fromStdString(loc_endpoint_str);
    // MOTOR STALL: matches this robot's mpc_controller launch tuning to the
    // sim-side threshold via the shared override written above (empty when
    // disabled or when writing it failed -- no flag passed either way).
    if (!stall_robot_spec_path.empty())
      ctl_args << "--robot-spec" << QString::fromStdString(stall_robot_spec_path);
    QProcess *ctl_proc = spawn_wrapped_process(this, controller_exe, ctl_args);

    ChildProc cp;
    cp.robot_name = robot->name;
    cp.handshake_port = handshake_port;
    cp.sim = sim_proc; // nullptr in real_mode -- every teardown/tick loop
                        // already null-checks cp.sim before touching it.
    cp.controller = ctl_proc;
    children_.push_back(cp);

    loc_endpoints.emplace_back(robot->name, loc_endpoint_str);
    robot_names.push_back(robot->name);
    handshake_ports.push_back(handshake_port);
    trajectories.push_back(traj);
  }

  localization_.start_with_endpoints(loc_endpoints);

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    state_ = RunState::Running;
    label_ = scenario->label() + label_suffix;
    error_reason_.clear();
    t0_valid_ = false;
    max_plan_time_ = scenario->max_time();
    handshake_failed_ = false;
    handshake_failure_reason_.clear();
    // FEATURE 2B: a freshly-started run is never paused, and carries no
    // stale pause accounting or pause/resume error from a previous run.
    paused_ = false;
    pause_accum_ = 0.0;
    last_error_.clear();
  }

  handshake_thread_ = std::thread(&ExecutionManager::run_handshake_thread, this, scenario,
                                   robot_names, handshake_ports, trajectories);
  return true;
}

void ExecutionManager::run_handshake_thread(
    std::shared_ptr<ScenarioModel> /*scenario*/, std::vector<std::string> robot_names,
    std::vector<int> handshake_ports, std::vector<ReloPush::trajectory> trajectories)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(kHandshakeReadinessDelayMs));
  if (handshake_stop_requested_.load())
    return;

  try
  {
    zmq::context_t ctx(1);
    std::vector<zmq::socket_t> socks;
    socks.reserve(robot_names.size());

    for (size_t i = 0; i < robot_names.size(); ++i)
    {
      zmq::socket_t sock(ctx, zmq::socket_type::req);
      sock.set(zmq::sockopt::linger, 0);
      sock.set(zmq::sockopt::sndtimeo, kHandshakePerAttemptMs);
      sock.connect("tcp://127.0.0.1:" + std::to_string(handshake_ports[i]));
      socks.push_back(std::move(sock));
    }

    // Phase 1: upload trajectories, wait ACK_RECEIVE_<robot>.
    for (size_t i = 0; i < robot_names.size(); ++i)
    {
      if (handshake_stop_requested_.load())
        return;
      const std::string encoded = base64_encode(trajectories[i].serialize());
      std::string reply, err;
      if (!send_and_wait_reply(socks[i], encoded, "ACK_RECEIVE_" + robot_names[i], &reply, &err,
                                &handshake_stop_requested_))
      {
        if (handshake_stop_requested_.load())
          return;
        fail_handshake(robot_names[i] + ": trajectory upload failed (" + err + ")");
        return;
      }
    }

    // Phase 2: broadcast START to every robot FIRST -- a separate loop from
    // Phase 3's ACK collection below, so a slow/unresponsive robot's ACK
    // wait cannot delay when a later robot's START is actually sent. This
    // mirrors run_on_robots_pipeline's own two-loop split (SearchOrchestrator
    // .cpp) specifically so robots begin executing near-simultaneously,
    // which the whole point of MARS's multi-robot coordination depends on.
    for (size_t i = 0; i < robot_names.size(); ++i)
    {
      if (handshake_stop_requested_.load())
        return;
      std::string err;
      if (!send_only(socks[i], "START", &err))
      {
        fail_handshake(robot_names[i] + ": START send failed (" + err + ")");
        return;
      }
    }

    // Phase 3: collect ACK_START_<robot> from all (this loop only waits --
    // never resends -- so it stays safe against the REQ socket's strict
    // send/recv alternation; see wait_for_reply()'s doc comment).
    for (size_t i = 0; i < robot_names.size(); ++i)
    {
      if (handshake_stop_requested_.load())
        return;
      std::string reply, err;
      if (!wait_for_reply(socks[i], "ACK_START_" + robot_names[i], &reply, &err,
                           &handshake_stop_requested_))
      {
        if (handshake_stop_requested_.load())
          return;
        fail_handshake(robot_names[i] + ": START handshake failed (" + err + ")");
        return;
      }
    }

    if (handshake_stop_requested_.load())
      return;

    std::lock_guard<std::mutex> lk(state_mutex_);
    // Another thread (abort()/a subsequent start_execution()) may have
    // already moved us out of Running; don't stomp a newer run's t0.
    if (state_ == RunState::Running && !t0_valid_)
    {
      t0_ = std::chrono::steady_clock::now();
      t0_valid_ = true;
    }
  }
  catch (const std::exception &ex)
  {
    if (!handshake_stop_requested_.load())
      fail_handshake(std::string("exception during handshake: ") + ex.what());
  }
}

void ExecutionManager::fail_handshake(const std::string &reason)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  handshake_failed_ = true;
  handshake_failure_reason_ = reason;
}

void ExecutionManager::tick()
{
  bool need_finalize_err = false;
  bool need_finalize_done = false;
  std::string err_reason;
  bool paused = false;

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ != RunState::Running)
      return;

    paused = paused_;
    if (handshake_failed_)
    {
      need_finalize_err = true;
      err_reason = handshake_failure_reason_;
    }
  }

  // FEATURE C / MOTOR STALL: ~1Hz idempotent republish of the current
  // noise sigmas + stall config to every connected robot -- slow-joiner
  // insurance (a subscription established just after a publish would
  // otherwise miss it entirely, same PUB/SUB caveat as
  // LocalizationListener's late-joiner comment elsewhere in this file).
  // Runs unconditionally while Running (not gated on `paused` below): a
  // UI change made right as PAUSE takes effect should still reach the sim
  // promptly, and mpc_robot_sim itself keeps ticking (only
  // mpc_controller's solve loop pauses).
  {
    const auto now = std::chrono::steady_clock::now();
    if (!noise_last_publish_valid_ ||
        std::chrono::duration<double>(now - noise_last_publish_).count() >=
            kNoisePublishIntervalS)
    {
      publish_sim_config();
      noise_last_publish_ = now;
      noise_last_publish_valid_ = true;
    }
  }

  if (need_finalize_err)
  {
    finalize_run(true, err_reason);
    return;
  }

  // FEATURE 2B: completion cannot fire while paused -- the plan clock is
  // frozen (see plan_time()'s doc comment) and mpc_controller keeps running
  // (publishing stop payloads) rather than exiting, so neither the
  // elapsed-time nor the all-controllers-exited completion check below
  // should be evaluated at all until resume_all() un-freezes things.
  if (paused)
    return;

  bool all_controllers_exited = !children_.empty();
  for (const auto &cp : children_)
  {
    if (cp.controller && cp.controller->state() != QProcess::NotRunning)
    {
      all_controllers_exited = false;
      break;
    }
  }

  double elapsed = 0.0;
  bool t0_valid = false;
  double max_plan_time = 0.0;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    t0_valid = t0_valid_;
    max_plan_time = max_plan_time_;
    if (t0_valid_)
      elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count() -
                pause_accum_;
  }

  if (t0_valid && elapsed > max_plan_time + kCompletionMarginS)
    need_finalize_done = true;
  else if (t0_valid && all_controllers_exited)
    need_finalize_done = true;

  if (need_finalize_done)
    finalize_run(false, "");
}

void ExecutionManager::terminate_children()
{
  localization_.stop();

  // FEATURE C: tear down this run's noise-config PUB sockets/context
  // alongside its children -- start_execution() creates a fresh set for the
  // next run.
  noise_pub_targets_.clear();
  noise_pub_ctx_.reset();
  noise_last_publish_valid_ = false;

  for (auto &cp : children_)
  {
    for (QProcess *proc : {cp.sim, cp.controller})
    {
      if (!proc)
        continue;
      if (proc->state() == QProcess::NotRunning)
        continue;
      proc->terminate();
    }
  }
  // Grace period for SIGTERM, then SIGKILL fallback -- mirrors
  // MPC/tests/test_mpc_sim_loop.cpp's ProcessGuard::terminate().
  for (auto &cp : children_)
  {
    for (QProcess *proc : {cp.sim, cp.controller})
    {
      if (!proc)
        continue;
      if (!proc->waitForFinished(kTerminateGraceMs))
      {
        proc->kill();
        proc->waitForFinished(kTerminatePollMs * 50);
      }
    }
  }
  for (auto &cp : children_)
  {
    if (cp.sim)
      cp.sim->deleteLater();
    if (cp.controller)
      cp.controller->deleteLater();
  }
  children_.clear();
}

void ExecutionManager::finalize_run(bool as_error, const std::string &reason)
{
  // See handshake_stop_requested_'s doc comment: bounds the join below to
  // roughly one retry attempt's timeout instead of the full handshake
  // budget, so tick() (which calls finalize_run() synchronously) stays
  // responsive.
  handshake_stop_requested_.store(true);
  terminate_children();
  if (handshake_thread_.joinable())
    handshake_thread_.join();

  std::lock_guard<std::mutex> lk(state_mutex_);
  state_ = as_error ? RunState::Err : RunState::Done;
  if (as_error)
    error_reason_ = reason;
}

void ExecutionManager::abort()
{
  handshake_stop_requested_.store(true);
  terminate_children();
  if (handshake_thread_.joinable())
    handshake_thread_.join();

  std::lock_guard<std::mutex> lk(state_mutex_);
  state_ = RunState::Idle;
  error_reason_.clear();
  t0_valid_ = false;
  // FEATURE 2B: a torn-down run is never paused.
  paused_ = false;
  pause_accum_ = 0.0;
  last_error_.clear();
}

void ExecutionManager::fail_externally(const std::string &label, const std::string &reason)
{
  // See this method's doc comment in SimVizCore.h: defensive teardown, not
  // expected to find anything active in the STAGING PHASE caller's normal
  // use (Idle/Done between runs), but stays correct if it ever is.
  handshake_stop_requested_.store(true);
  terminate_children();
  if (handshake_thread_.joinable())
    handshake_thread_.join();
  handshake_stop_requested_.store(false);

  std::lock_guard<std::mutex> lk(state_mutex_);
  state_ = RunState::Err;
  label_ = label;
  error_reason_ = reason;
  t0_valid_ = false;
  paused_ = false;
  pause_accum_ = 0.0;
  last_error_.clear();
}

RunState ExecutionManager::state() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return state_;
}

std::string ExecutionManager::label() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return label_;
}

// LAG READOUT (three-clock lag-diagnosis task): see SimVizCore.h's doc
// comment for the full spec. Pure function -- only touches its arguments.
double progress_time_search(const TimeTable &timetable, EntityMeta *ent, double x, double y,
                             double center, double window_half_s, double step_s,
                             double tie_eps_m)
{
  const double lo_t = center - window_half_s;
  const double hi_t = center + window_half_s;
  const int n_steps = std::max(1, static_cast<int>(std::lround((hi_t - lo_t) / step_s)));

  double best_d = std::numeric_limits<double>::infinity();
  std::vector<double> candidate_times;
  std::vector<double> candidate_dists;
  candidate_times.reserve(n_steps + 1);
  candidate_dists.reserve(n_steps + 1);
  for (int i = 0; i <= n_steps; ++i)
  {
    const double s = lo_t + (hi_t - lo_t) * static_cast<double>(i) / static_cast<double>(n_steps);
    const Pose p = timetable.get_pose(ent, s);
    const double d = std::hypot(p.x - x, p.y - y);
    candidate_times.push_back(s);
    candidate_dists.push_back(d);
    if (d < best_d)
      best_d = d;
  }

  // Among near-ties (within tie_eps_m of the best distance -- the hallmark
  // of a hold, see doc comment), pick whichever candidate is closest to
  // `center`.
  double best_s = center;
  double best_center_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < candidate_times.size(); ++i)
  {
    if (candidate_dists[i] > best_d + tie_eps_m)
      continue;
    const double center_dist = std::fabs(candidate_times[i] - center);
    if (center_dist < best_center_dist)
    {
      best_center_dist = center_dist;
      best_s = candidate_times[i];
    }
  }
  return best_s;
}

std::optional<double> ExecutionManager::lag_seconds(const std::string &robot_name) const
{
  if (state() != RunState::Running)
    return std::nullopt;
  std::shared_ptr<ScenarioModel> model = scenario_model();
  if (!model)
    return std::nullopt;
  auto live = localization_.latest_pose(robot_name);
  if (!live || !live->has_pose)
    return std::nullopt;

  EntityMeta *ent = nullptr;
  for (EntityMeta *r : model->robots_sorted_by_name())
  {
    if (r && r->name == robot_name)
    {
      ent = r;
      break;
    }
  }
  if (!ent)
    return std::nullopt;

  const double t_viz = plan_time();
  // BUGFIX (found via a deliberate severe-CPU-starvation stress test, see
  // the three-clock lag-diagnosis report): progress_time_search()'s window
  // is intentionally fixed-width (see its own doc comment / unit tests) --
  // but a search that SATURATES at either edge (the true nearest-pose match
  // lies outside the window) silently reports a lag CLAMPED to
  // window_half_s, which looks like a stable, precise reading even though
  // the real value could be far larger and still growing. Observed live
  // under that stress test: reported lag plateaued at "8.00"/"7.80" for
  // many consecutive seconds while the robot was, in fact, still falling
  // further behind. Detect saturation (matched time within one grid step of
  // either edge) and retry ONCE with a much wider window before returning,
  // so a genuinely large lag is reported close to its true size instead of
  // silently capped -- normal on-schedule/brief-transient cases (the
  // overwhelming majority of calls) never saturate the first, narrow
  // search, so this retry essentially never fires in practice and costs
  // nothing extra then.
  constexpr double kWindowHalfS = 8.0;
  constexpr double kStepS = 0.2;
  double t_prog = progress_time_search(model->timetable(), ent, live->pose.x, live->pose.y,
                                        t_viz, kWindowHalfS, kStepS);
  const bool saturated_low = std::fabs(t_prog - (t_viz - kWindowHalfS)) < kStepS;
  const bool saturated_high = std::fabs(t_prog - (t_viz + kWindowHalfS)) < kStepS;
  if (saturated_low || saturated_high)
  {
    constexpr double kWideWindowHalfS = 60.0;
    constexpr double kWideStepS = 0.5;
    t_prog = progress_time_search(model->timetable(), ent, live->pose.x, live->pose.y, t_viz,
                                   kWideWindowHalfS, kWideStepS);
  }
  return t_viz - t_prog;
}

double ExecutionManager::plan_time() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  if (!t0_valid_)
    return 0.0;
  const auto now = std::chrono::steady_clock::now();
  // FEATURE 2B: pause-adjusted (frozen-during-pause) plan clock -- subtract
  // every COMPLETED pause's duration (pause_accum_), and, while currently
  // paused, ALSO subtract the in-progress pause's duration so far. The
  // result is independent of `now` while paused (frozen at the value it had
  // the instant pause_all() succeeded), mirroring
  // mpc::compute_effective_elapsed's formula on the controller side.
  double adjusted = std::chrono::duration<double>(now - t0_).count() - pause_accum_;
  if (paused_)
    adjusted -= std::chrono::duration<double>(now - pause_started_at_).count();
  return adjusted;
}

std::string ExecutionManager::error_reason() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return error_reason_;
}

bool ExecutionManager::is_paused() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return paused_;
}

std::string ExecutionManager::last_error() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return last_error_;
}

bool ExecutionManager::send_pause_resume(bool pause)
{
  std::vector<std::pair<std::string, int>> targets; // (robot_name, handshake_port)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ != RunState::Running)
    {
      last_error_ = "not running";
      return false;
    }
    // state_ flips to Running SYNCHRONOUSLY inside start_execution() (see its
    // doc comment: "the caller may reply ACK_EXECUTE immediately without
    // waiting for the handshake to finish") -- well before the background
    // handshake thread has actually uploaded any robot's trajectory or
    // collected its ACK_START (t0_valid_ only becomes true once that fully
    // succeeds). Sending PAUSE/RESUME to a handshake port before then would
    // race mpc_controller's still-pending FIRST recv() (which unconditionally
    // treats whatever bytes arrive there as the base64-encoded trajectory
    // payload) -- fail fast with a retryable error instead.
    if (!t0_valid_)
    {
      last_error_ = "handshake not complete yet";
      return false;
    }
    targets.reserve(children_.size());
    for (const auto &cp : children_)
      targets.emplace_back(cp.robot_name, cp.handshake_port);
  }

  const std::string msg = pause ? "PAUSE" : "RESUME";
  const std::string expect_prefix = pause ? "ACK_PAUSE_" : "ACK_RESUME_";
  bool all_ok = true;
  std::string failures;

  try
  {
    zmq::context_t ctx(1);
    for (const auto &[name, port] : targets)
    {
      bool ok = false;
      std::string this_failure;
      try
      {
        zmq::socket_t sock(ctx, zmq::socket_type::req);
        sock.set(zmq::sockopt::linger, 0);
        sock.set(zmq::sockopt::rcvtimeo, kPauseResumeTimeoutMs);
        sock.set(zmq::sockopt::sndtimeo, kPauseResumeTimeoutMs);
        sock.connect("tcp://127.0.0.1:" + std::to_string(port));
        const auto send_res = sock.send(zmq::buffer(msg), zmq::send_flags::none);
        if (!send_res)
        {
          this_failure = "send timed out";
        }
        else
        {
          zmq::message_t reply;
          const auto recv_res = sock.recv(reply, zmq::recv_flags::none);
          if (!recv_res)
          {
            this_failure = "recv timed out";
          }
          else
          {
            std::string s(static_cast<const char *>(reply.data()), reply.size());
            if (s.rfind(expect_prefix, 0) == 0)
              ok = true;
            else
              this_failure = "unexpected reply '" + s + "'";
          }
        }
      }
      catch (const std::exception &ex)
      {
        this_failure = ex.what();
      }
      if (!ok)
      {
        all_ok = false;
        failures += name + ": " + this_failure + "; ";
      }
    }
  }
  catch (const std::exception &ex)
  {
    all_ok = false;
    failures += std::string("zmq context error: ") + ex.what() + "; ";
  }

  std::lock_guard<std::mutex> lk(state_mutex_);
  if (!all_ok)
  {
    last_error_ = (pause ? "pause failed: " : "resume failed: ") + failures;
    return false;
  }

  last_error_.clear();
  if (pause)
  {
    if (!paused_)
    {
      paused_ = true;
      pause_started_at_ = std::chrono::steady_clock::now();
    }
  }
  else
  {
    if (paused_)
    {
      pause_accum_ +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - pause_started_at_)
              .count();
      paused_ = false;
    }
  }
  return true;
}

bool ExecutionManager::pause_all()
{
  return send_pause_resume(true);
}

bool ExecutionManager::resume_all()
{
  return send_pause_resume(false);
}

// ---------------------------------------------------------------------
// FEATURE C (motor-noise sliders: ACCEL channel + PART (1) STEER channel)
// + MOTOR STALL (min-speed deadband UI controls).
// ---------------------------------------------------------------------

// Renamed from FEATURE C's original publish_noise_config(): now also
// carries MOTOR STALL's "deadband"/"min_moving_speed"/"min_sustain_speed"
// fields on the SAME "/<robot>/sim_config" payload/socket set (see
// NoisePubTarget's doc comment in SimVizCore.h -- the socket plumbing
// itself is unchanged).
void ExecutionManager::publish_sim_config()
{
  double sigma;
  double steer_sigma;
  bool stall_on;
  double stall_level;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    sigma = noise_sigma_pct_;
    steer_sigma = steer_noise_sigma_pct_;
    stall_on = stall_enabled_;
    stall_level = stall_level_;
  }
  // ANTI-RE-STALL HYSTERESIS: same 0.7*level sustain fraction as
  // start_execution()'s spawn-time override -- see spawn_stall_sustain's
  // doc comment there for the rationale (kinetic < static friction). This
  // is the "override" half of "spawn/override sustain = 0.7*level": a live
  // slider change re-publishes through here, so it must derive the SAME
  // sustain the spawn-time write used, keeping sim and controller
  // calibrated to the one user-facing "stall level" throughout a run, not
  // just at spawn.
  const double stall_sustain = 0.7 * stall_level;

  // Plain JSON, matching mpc::parse_sim_config_payload()'s expected wire
  // format ({"noise_sigma_pct": <number>, "steer_noise_sigma_pct":
  // <number>, ...}, NOT base64) -- PLUS the MOTOR STALL fields
  // ("deadband": 0|1, "min_moving_speed"/"min_sustain_speed": <number>)
  // documented in this task's CONTRACT 2. ALL fields are always sent
  // together on every publish (from set_noise_sigma_pct()/
  // set_steer_noise_sigma_pct()/set_stall_enabled()/set_stall_level(), or
  // the ~1Hz republish) -- same "every field, every time" discipline the
  // original noise-only publish already used, now just covering more
  // fields. The receiving mpc_robot_sim parses each key independently and
  // ignores unknown/absent ones, so this is a convenience for staying in
  // sync, not a protocol requirement.
  std::ostringstream payload;
  payload << "{\"noise_sigma_pct\":" << sigma << ",\"steer_noise_sigma_pct\":" << steer_sigma
          << ",\"deadband\":" << (stall_on ? 1 : 0) << ",\"min_moving_speed\":" << stall_level
          << ",\"min_sustain_speed\":" << stall_sustain << "}";
  const std::string payload_str = payload.str();

  for (auto &target : noise_pub_targets_)
  {
    if (!target.sock)
      continue;
    const std::string topic = "/" + target.robot_name + "/sim_config";
    try
    {
      zmq::message_t topic_msg(topic.begin(), topic.end());
      zmq::message_t payload_msg(payload_str.begin(), payload_str.end());
      target.sock->send(topic_msg, zmq::send_flags::sndmore);
      target.sock->send(payload_msg, zmq::send_flags::none);
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[mars_sim_viz] Warning: failed to publish sim config to '"
                 << target.robot_name << "': " << ex.what() << std::endl;
    }
  }
}

void ExecutionManager::set_noise_sigma_pct(double sigma_pct)
{
  const double clamped = clamp_noise_sigma_pct(sigma_pct);
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    noise_sigma_pct_ = clamped;
  }
  // Only meaningful if a run is currently active (noise_pub_targets_ is
  // populated by start_execution(), cleared by terminate_children()) --
  // otherwise this just updates what the NEXT spawn will use, with nothing
  // to publish to yet. tick()'s ~1Hz republish also covers this value going
  // forward for as long as the run stays active.
  if (!noise_pub_targets_.empty())
    publish_sim_config();
}

double ExecutionManager::noise_sigma_pct() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return noise_sigma_pct_;
}

// PART (1) STEERING-NOISE SPLIT: mirrors set_noise_sigma_pct()/
// noise_sigma_pct() above for the independent STEER channel.
void ExecutionManager::set_steer_noise_sigma_pct(double sigma_pct)
{
  const double clamped = clamp_noise_sigma_pct(sigma_pct);
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    steer_noise_sigma_pct_ = clamped;
  }
  if (!noise_pub_targets_.empty())
    publish_sim_config();
}

double ExecutionManager::steer_noise_sigma_pct() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return steer_noise_sigma_pct_;
}

// MOTOR STALL: mirrors set_noise_sigma_pct()/noise_sigma_pct() above for
// the stall-enable flag.
void ExecutionManager::set_stall_enabled(bool enabled)
{
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    stall_enabled_ = enabled;
  }
  if (!noise_pub_targets_.empty())
    publish_sim_config();
}

bool ExecutionManager::stall_enabled() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return stall_enabled_;
}

// MOTOR STALL: mirrors set_noise_sigma_pct()/noise_sigma_pct() above for
// the stall LEVEL (m/s, used for both min_moving_speed/min_sustain_speed).
void ExecutionManager::set_stall_level(double level_mps)
{
  const double clamped = clamp_stall_level(level_mps);
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    stall_level_ = clamped;
  }
  if (!noise_pub_targets_.empty())
    publish_sim_config();
}

double ExecutionManager::stall_level() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return stall_level_;
}

// ===========================================================================
// SimVizManager
// ===========================================================================

SimVizManager::SimVizManager(SimVizConfig config, QObject *parent)
    : QObject(parent), config_(config), exec_mgr_(config, this), mocap_(config.mocap, this)
{
  // UNMAPPED ROBOTS ARE SIM-ONLY (mixed real/sim fleet): opts exec_mgr_ into
  // the per-robot real-vs-sim split for every future start_execution() call
  // -- see ExecutionManager::set_mocap_source()'s doc comment. mocap_ is a
  // member of this same SimVizManager and outlives exec_mgr_ (both are
  // torn down together), so this pointer stays valid for exec_mgr_'s whole
  // lifetime.
  exec_mgr_.set_mocap_source(&mocap_);
}

void SimVizManager::start()
{
  control_.bind(config_.control_port);

  // PART B, TEST-ONLY: see SimVizConfig::stage_real_use_localization_
  // source's doc comment -- started once, here, so it is already tracking
  // poses by the time a test sends STAGE_REAL (production leaves this
  // idle; stage_real_current_pose_of() reads MocapManager instead).
  if (config_.stage_real_use_localization_source)
  {
    std::vector<std::pair<std::string, std::string>> endpoints;
    endpoints.reserve(config_.real_robot_endpoints.size());
    for (const auto &[name, ep] : config_.real_robot_endpoints)
    {
      const std::string loc = ep.localization_endpoint.empty()
                                   ? ("tcp://127.0.0.1:" +
                                      std::to_string(config_.mocap.loc_port_start))
                                   : ep.localization_endpoint;
      endpoints.emplace_back(name, loc);
    }
    real_pose_listener_.start_with_endpoints(endpoints);
  }
}

void SimVizManager::tick()
{
  auto request = control_.poll_once();
  if (request)
  {
    const std::string reply = handle_command(*request);
    control_.reply(reply);
  }
  // STAGING PHASE: drive the staging sequence (if any) forward before
  // ticking exec_mgr_ -- tick_staging() reads exec_mgr_'s CURRENT state
  // (from the previous tick) to decide whether a leg just finished, then
  // (at most) starts the next leg/main run; exec_mgr_.tick() right after
  // observes whatever that just started from t=0 rather than being one
  // full tick further along.
  tick_staging();
  // PART B: mutually exclusive with tick_staging() above -- stage_real_
  // robots()/start_execution_with_staging() each refuse to start while the
  // other sequence (or a normal run) is active, so at most one of
  // tick_staging()/tick_stage_real() ever does real work on a given tick.
  tick_stage_real();
  exec_mgr_.tick();
  // OptiTrack/MocapManager: additive, independent of the execution/staging
  // state machines above -- see MocapCore.h's tick() doc comment.
  mocap_.tick();
}

std::string SimVizManager::handle_command(const std::string &request)
{
  if (request == "PING")
    return "PONG mars_sim_viz v1";

  if (request == "STATUS")
  {
    // STAGING PHASE: takes priority over exec_mgr_'s own state -- while a
    // staging sequence is active, exec_mgr_ is really running (or between)
    // single-robot LEGS, whose own RUNNING/DONE/ERR text would be
    // confusing (and briefly wrong: a leg's own label_ carries a
    // "__staging_<robot>_a<n>" suffix, see ExecutionManager::
    // start_execution()'s label_suffix parameter) to surface directly.
    // This is a brand-new status-string prefix ("STAGING", never emitted
    // unless config_.staging_enabled was true for this EXECUTE), so it
    // cannot collide with any existing caller's IDLE/RUNNING/PAUSED/DONE/
    // ERR prefix or exact-equality check.
    if (staging_.active)
    {
      std::ostringstream oss;
      const size_t total = staging_.robots.size();
      const size_t idx = std::min(staging_.current_index, total == 0 ? 0 : total - 1);
      const std::string robot_name =
          (total == 0) ? std::string("?") : staging_.robots[idx].name;
      oss << "STAGING " << robot_name << " (" << (idx + 1) << "/" << total << ")";
      return oss.str();
    }

    // PART B: "move real robots to initial poses" sequence -- same
    // reasoning/priority as the staging_.active branch immediately above
    // (mutually exclusive with it and with a normal run by construction,
    // see stage_real_robots()'s preconditions).
    if (stage_real_.active)
      return "STAGING_REAL " + stage_real_progress_text();

    // FEATURE C: additive "noise=A%/S%" suffix (A = accel channel, S =
    // steer channel; PART (1) STEERING-NOISE SPLIT). Deliberately NOT
    // applied to the bare "IDLE" reply below -- existing tests assert
    // STATUS == "IDLE" (exact equality, not a prefix check) in several
    // places (test_sim_viz_control.cpp, test_sim_viz_pause.cpp,
    // test_sim_viz_execute.cpp), which this task's NON-NEGOTIABLE rules
    // require to keep passing UNMODIFIED; every other branch's callers only
    // ever prefix-check ("RUNNING"/"PAUSED"/"DONE "/"ERR "), so appending a
    // suffix there is genuinely additive.
    auto with_noise_suffix = [this](const std::string &base) -> std::string
    {
      std::ostringstream oss;
      oss << base << " noise="
          << static_cast<int>(std::lround(exec_mgr_.noise_sigma_pct() * 100.0)) << "%/"
          << static_cast<int>(std::lround(exec_mgr_.steer_noise_sigma_pct() * 100.0)) << "%";
      return oss.str();
    };

    // MOTOR STALL: additive " stall=off"/" stall=<L>" suffix, layered AFTER
    // with_noise_suffix()'s output -- per this task's spec, applied to
    // RUNNING/PAUSED/DONE only (i.e. NOT IDLE -- see with_noise_suffix's own
    // doc comment on the exact-equality "IDLE" tests above -- and NOT ERR,
    // to keep that branch's format exactly as with_noise_suffix alone
    // already produces it). Callers of the RUNNING/PAUSED/DONE branches
    // only ever prefix-check ("RUNNING"/"PAUSED"/"DONE ") or substring-check
    // ("noise=A%/S%"), so appending here is genuinely additive, same
    // reasoning as with_noise_suffix's own addition.
    auto with_stall_suffix = [this](const std::string &base) -> std::string
    {
      std::ostringstream oss;
      oss << base << " stall=";
      if (exec_mgr_.stall_enabled())
        oss << exec_mgr_.stall_level();
      else
        oss << "off";
      return oss.str();
    };

    // LAG READOUT (three-clock lag-diagnosis task): additive " lag=<r1>:
    // <v1>,<r2>:<v2>,..." suffix, one entry per current scenario robot
    // (robots_sorted_by_name() order, matching every other per-robot
    // listing in this file) -- "NA" for a robot with no live pose yet
    // (ExecutionManager::lag_seconds()'s std::nullopt). Applied ONLY to the
    // RUNNING/PAUSED branch below (same scope lag_seconds() itself already
    // restricts to), so this is purely additive: every existing caller that
    // prefix-checks "RUNNING"/"PAUSED"/"DONE "/"ERR " or exact-equality-
    // checks "IDLE" keeps passing unmodified, same reasoning as
    // with_noise_suffix/with_stall_suffix above.
    auto with_lag_suffix = [this](const std::string &base) -> std::string
    {
      auto model = exec_mgr_.scenario_model();
      if (!model)
        return base;
      std::ostringstream oss;
      oss << base << " lag=";
      bool first = true;
      for (EntityMeta *robot : model->robots_sorted_by_name())
      {
        if (!robot)
          continue;
        if (!first)
          oss << ",";
        first = false;
        oss << robot->name << ":";
        auto lag = exec_mgr_.lag_seconds(robot->name);
        if (lag.has_value())
          oss << std::fixed << std::setprecision(2) << *lag;
        else
          oss << "NA";
      }
      return oss.str();
    };

    switch (exec_mgr_.state())
    {
    case RunState::Idle:
      return "IDLE";
    case RunState::Running:
    {
      // FEATURE 2B: STATUS distinguishes a live-but-paused run from a
      // normally-running one; plan_time() is already pause-adjusted (frozen
      // while paused_), so `t=` reads as the frozen time here with no extra
      // bookkeeping needed.
      std::ostringstream oss;
      oss << (exec_mgr_.is_paused() ? "PAUSED " : "RUNNING ") << exec_mgr_.label()
          << " t=" << exec_mgr_.plan_time();
      return with_lag_suffix(with_stall_suffix(with_noise_suffix(oss.str())));
    }
    case RunState::Done:
      return with_stall_suffix(with_noise_suffix("DONE " + exec_mgr_.label()));
    case RunState::Err:
      return with_noise_suffix("ERR " + exec_mgr_.label() + " " + exec_mgr_.error_reason());
    }
    return "ERR unknown-state";
  }

  if (request == "ABORT")
  {
    // STAGING PHASE: also cancel any in-progress staging sequence -- without
    // this, exec_mgr_.abort() alone leaves exec_mgr_ Idle (nothing will
    // ever move it to Done/Err again) while staging_.active stays true,
    // wedging STATUS on "STAGING ..." forever (tick_staging() would just
    // keep waiting on a leg that was already torn out from under it).
    // PART B: also cancel any in-progress STAGE_REAL sequence -- same
    // "wedged STATUS forever" reasoning as clearing staging_ immediately
    // above.
    staging_ = StagingSequence{};
    stage_real_ = StageRealSequence{};
    exec_mgr_.abort();
    return "ACK_ABORT";
  }

  if (request == "PAUSE")
    return pause();

  if (request == "RESUME")
    return resume();

  if (request == "RESTART")
    return restart();

  if (request.rfind("EXECUTE ", 0) == 0)
  {
    const std::string path = request.substr(std::string("EXECUTE ").size());
    return execute_scenario_file(path);
  }

  // ---------------------------------------------------------------------
  // PART B: "move real robots to initial poses" -- async, like EXECUTE
  // (see stage_real_scenario_file()'s doc comment). Bare "STAGE_REAL"
  // (no path) reuses the last scenario path, mirroring RESTART's own
  // "reload the last scenario" convention; "STAGE_REAL <path>" loads a
  // fresh one (and remembers it via note_scenario_path(), same as
  // execute_scenario_file()).
  // ---------------------------------------------------------------------
  if (request == "STAGE_REAL")
  {
    if (last_scenario_path_.empty())
      return "ERR no scenario";
    return stage_real_scenario_file(last_scenario_path_);
  }
  if (request.rfind("STAGE_REAL ", 0) == 0)
  {
    const std::string path = request.substr(std::string("STAGE_REAL ").size());
    return stage_real_scenario_file(path);
  }

  // ---------------------------------------------------------------------
  // OptiTrack/MocapManager (see MARS/src/simviz/MocapCore.h, DESIGN part A)
  // -- additive verbs, independent of the EXECUTE/PAUSE/RESUME/RESTART/
  // ABORT protocol above.
  // ---------------------------------------------------------------------
  if (request == "MOCAP_CONNECT")
  {
    std::string err;
    if (!mocap_.start_bridge(&err))
      return "ERR " + err;
    return "ACK";
  }

  if (request == "MOCAP_DISCONNECT")
  {
    mocap_.stop_bridge();
    return "ACK";
  }

  if (request == "MOCAP_STATUS")
  {
    std::ostringstream oss;
    oss << mocap_state_name(mocap_.state());
    if (mocap_.state() == MocapState::Error)
      oss << " " << mocap_.error_reason();

    oss << " bodies=";
    bool first = true;
    for (const auto &b : mocap_.bodies())
    {
      if (!first)
        oss << ",";
      first = false;
      oss << b.published_name << ":" << std::fixed << std::setprecision(2) << b.age_s;
    }

    // mapped=<k/N>: k = current-scenario robots with any live mocap pose
    // recorded (regardless of freshness -- MOCAP_STATUS's own per-body
    // age= list already conveys freshness), N = total scenario robots. 0/0
    // when no scenario is loaded.
    size_t mapped = 0, total = 0;
    auto model = exec_mgr_.scenario_model();
    if (model)
    {
      auto robots = model->robots_sorted_by_name();
      total = robots.size();
      for (EntityMeta *r : robots)
      {
        if (r && mocap_.body_pose(r->name).has_value())
          ++mapped;
      }
    }
    oss << " mapped=" << mapped << "/" << total;
    return oss.str();
  }

  return "ERR unknown command";
}

std::string SimVizManager::pause()
{
  if (exec_mgr_.state() != RunState::Running)
    return "ERR not running";
  if (!exec_mgr_.pause_all())
    return "ERR " + exec_mgr_.last_error();
  return "ACK_PAUSE";
}

std::string SimVizManager::resume()
{
  if (exec_mgr_.state() != RunState::Running)
    return "ERR not running";
  if (!exec_mgr_.resume_all())
    return "ERR " + exec_mgr_.last_error();
  return "ACK_RESUME";
}

std::string SimVizManager::restart()
{
  if (last_scenario_path_.empty())
    return "ERR no scenario";
  // Teardown path -- mirrors ABORT (ExecutionManager::abort() kills/reaps
  // every spawned child and returns to Idle) so execute_scenario_file()
  // below spawns a genuinely fresh set of processes rather than colliding
  // with ones still shutting down. STAGING PHASE: also cancel any
  // in-progress staging sequence, same reasoning as the ABORT branch in
  // handle_command() above (a RESTART mid-staging must not leave
  // staging_.active true, else execute_scenario_file()'s "ERR busy" check
  // below would reject the reload it is supposed to perform). PART B: same
  // reasoning for stage_real_.
  staging_ = StagingSequence{};
  stage_real_ = StageRealSequence{};
  exec_mgr_.abort();
  const std::string reply = execute_scenario_file(last_scenario_path_);
  if (reply == "ACK_EXECUTE")
    return "ACK_RESTART";
  return reply; // forward "ERR ..." verbatim
}

std::string SimVizManager::execute_scenario_file(const std::string &path)
{
  // PART B: also refuse while a STAGE_REAL sequence is in progress -- same
  // "would otherwise collide with exec_mgr_" reasoning as the staging_
  // check.
  if (exec_mgr_.is_busy() || staging_.active || stage_real_.active)
    return "ERR busy";

  std::shared_ptr<ScenarioModel> model;
  try
  {
    model = std::make_shared<ScenarioModel>(ScenarioModel::load_from_file(path));
  }
  catch (const std::exception &ex)
  {
    return std::string("ERR ") + ex.what();
  }

  std::string err;
  if (!start_execution_with_staging(model, &err))
    return "ERR " + (err.empty() ? std::string("busy") : err);

  // FEATURE 2C: remember this path so restart() can reload + re-execute the
  // same scenario later.
  note_scenario_path(path);

  return "ACK_EXECUTE";
}

// ===========================================================================
// STAGING PHASE
// ===========================================================================

Pose SimVizManager::staging_start_pose_for(EntityMeta *robot,
                                            const ReloPush::trajectory &traj) const
{
  // Reproduces ExecutionManager::start_execution()'s own main-run spawn-pose
  // rule (first trajectory waypoint, else initial_pose) so a staging leg's
  // TARGET is exactly where the main run will actually expect this robot to
  // be, not merely its (possibly stale, pre-plan) initial_pose.
  if (traj.trajectory_points && !traj.trajectory_points->empty())
  {
    const auto &wp0 = traj.trajectory_points->front();
    return Pose(wp0.x, wp0.y, wp0.yaw);
  }
  return robot->initial_pose;
}

bool SimVizManager::start_execution_with_staging(std::shared_ptr<ScenarioModel> scenario,
                                                  std::string *err_out)
{
  // A staging sequence is itself "busy" even if exec_mgr_ momentarily is
  // not (e.g. between two legs) -- check this FIRST, unconditionally.
  if (staging_.active)
  {
    if (err_out)
      *err_out = "staging in progress";
    return false;
  }
  // PART B: a STAGE_REAL sequence is "busy" for the exact same reason.
  if (stage_real_.active)
  {
    if (err_out)
      *err_out = "stage_real in progress";
    return false;
  }

  if (!config_.staging_enabled)
  {
    // Pure passthrough -- byte-identical to calling
    // exec_mgr_.start_execution(scenario) directly (every one of this
    // task's 12 regression suites, and every pre-existing call site, hits
    // this branch, since none of them ever set staging_enabled).
    return exec_mgr_.start_execution(scenario);
  }

  if (exec_mgr_.is_busy())
  {
    if (err_out)
      *err_out = "busy";
    return false;
  }

  const auto robots = scenario->robots_sorted_by_name();
  if (robots.empty())
  {
    // Let ExecutionManager report its own well-known "scenario has no
    // robot entities" Err (unchanged wording/behavior) rather than
    // duplicating that message here.
    return exec_mgr_.start_execution(scenario);
  }

  std::vector<std::string> all_names;
  all_names.reserve(robots.size());
  for (EntityMeta *r : robots)
    all_names.push_back(r->name);

  // Presence determination: TEST-ONLY offsets mode when configured, else
  // "hardware mode" via live localization freshness (source-agnostic --
  // see LocalizationListener::has_fresh_pose()'s doc comment). Note that in
  // hardware mode, presence can only ever be detected for a robot that has
  // ALREADY published at least one localization sample on the ports
  // exec_mgr_'s LocalizationListener is listening on -- today that listener
  // is only started by ExecutionManager::start_execution() itself (i.e.
  // once a run/leg is already active), so this branch is a no-op (every
  // robot MISSING) until a persistent, pre-EXECUTE localization source
  // (e.g. a real mocap bridge) is wired up; test-offsets mode is what
  // exercises the STAGING PHASE end-to-end today.
  std::vector<std::string> present_names;
  std::vector<std::string> missing_names;
  std::unordered_map<std::string, Pose> measured;

  if (!config_.staging_test_offsets.empty())
  {
    const auto offsets = staging::parse_staging_test_offsets(config_.staging_test_offsets);
    const auto presence = staging::filter_presence_by_test_offsets(all_names, offsets);
    present_names = presence.present;
    missing_names = presence.missing;
    for (EntityMeta *r : robots)
    {
      auto it = offsets.find(r->name);
      if (it != offsets.end())
        measured[r->name] = staging::apply_test_offset(r->initial_pose, it->second);
    }
  }
  else
  {
    for (EntityMeta *r : robots)
    {
      if (exec_mgr_.localization().has_fresh_pose(r->name, config_.staging_localization_wait_s))
      {
        present_names.push_back(r->name);
        auto lp = exec_mgr_.localization().latest_pose(r->name);
        measured[r->name] = lp ? lp->pose : r->initial_pose;
      }
      else
      {
        missing_names.push_back(r->name);
      }
    }
  }

  if (present_names.empty())
  {
    if (err_out)
      *err_out = "staging: no robots present";
    return false;
  }

  StagingSequence seq;
  seq.scenario = scenario;
  seq.missing = missing_names;
  seq.active = true;

  std::unordered_map<std::string, ReloPush::trajectory> traj_by_name;
  for (const auto &pr : scenario->robot_trajectories())
  {
    if (pr.first)
      traj_by_name[pr.first->name] = pr.second;
  }

  for (EntityMeta *r : robots)
  {
    if (std::find(present_names.begin(), present_names.end(), r->name) == present_names.end())
      continue;
    RobotMeta *robot_meta = dynamic_cast<RobotMeta *>(r);
    if (!robot_meta)
      continue; // defensive: robots_sorted_by_name() only ever returns EntityType::ROBOT entries.

    StagingRobotState st;
    st.name = r->name;
    st.robot = robot_meta;
    ReloPush::trajectory empty_traj;
    const auto traj_it = traj_by_name.find(r->name);
    st.start_pose =
        staging_start_pose_for(r, traj_it != traj_by_name.end() ? traj_it->second : empty_traj);
    auto measured_it = measured.find(r->name);
    st.measured_pose = measured_it != measured.end() ? measured_it->second : st.start_pose;
    seq.robots.push_back(std::move(st));
  }

  if (seq.robots.empty())
  {
    // Defensive only -- every present_names entry failed the RobotMeta
    // cast above, which cannot happen given how every scenario in this
    // codebase constructs its ROBOT entities. Fail loudly rather than
    // falling through to tick_staging()'s "all present robots done" branch
    // with an EMPTY subset, which ExecutionManager::start_execution()
    // would (correctly, for its OTHER callers) interpret as "no
    // filtering -- run every robot in the scenario", silently defeating
    // the presence exclusion this whole method exists to enforce.
    if (err_out)
      *err_out = "staging: no present robot resolved to a RobotMeta entity";
    return false;
  }

  if (!seq.missing.empty())
  {
    std::ostringstream oss;
    oss << "[mars_sim_viz] STAGING: robot(s) [";
    for (size_t i = 0; i < seq.missing.size(); ++i)
      oss << (i ? ", " : "") << seq.missing[i];
    oss << "] not present -- excluded from staging and the main run";
    std::cout << oss.str() << std::endl;
  }

  staging_ = std::move(seq);
  return true;
}

void SimVizManager::fail_staging(const std::string &reason)
{
  const std::string label = staging_.scenario ? staging_.scenario->label() : std::string();
  staging_ = StagingSequence{};
  exec_mgr_.fail_externally(label, reason);
}

void SimVizManager::tick_staging()
{
  if (!staging_.active)
    return;

  if (staging_.waiting_for_leg)
  {
    const RunState st = exec_mgr_.state();
    if (st == RunState::Running)
      return; // leg still in flight.

    if (st == RunState::Err)
    {
      const std::string &name = staging_.robots[staging_.current_index].name;
      fail_staging("staging leg for '" + name + "' failed to execute: " +
                   exec_mgr_.error_reason());
      return;
    }

    if (st != RunState::Done)
      return; // Idle -- shouldn't happen mid-leg; wait rather than misbehave.

    // Leg finished -- capture the achieved pose BEFORE any further
    // start_execution() call (next leg or the main run) resets
    // localization (ExecutionManager::start_execution()'s unconditional
    // localization_.reset() at its top -- see this class's doc comment in
    // SimVizCore.h for the GOTCHA this avoids).
    StagingRobotState &cur = staging_.robots[staging_.current_index];
    auto lp = exec_mgr_.localization().latest_pose(cur.name);
    if (lp)
      cur.measured_pose = lp->pose;
    cur.attempts_done++;
    staging_.waiting_for_leg = false;
    return; // re-decide (skip-now-verified / retry / fail) on the next tick.
  }

  if (staging_.current_index >= staging_.robots.size())
  {
    // Every present robot is at (or within tolerance of) its start pose --
    // run the MAIN scenario, present-robot subset only. Clear staging_
    // BEFORE calling start_execution() so a STATUS poll racing this exact
    // call never observes a stale "STAGING ..." string once the real run
    // has actually started.
    std::vector<std::string> subset;
    subset.reserve(staging_.robots.size());
    for (const auto &r : staging_.robots)
      subset.push_back(r.name);
    std::shared_ptr<ScenarioModel> scenario = staging_.scenario;
    staging_ = StagingSequence{};
    exec_mgr_.start_execution(scenario, subset);
    return;
  }

  StagingRobotState &cur = staging_.robots[staging_.current_index];
  const staging::Decision decision = staging::staging_decision(
      cur.measured_pose, cur.start_pose, config_.staging_pos_tol, config_.staging_yaw_tol);
  if (decision == staging::Decision::Skip)
  {
    staging_.current_index++;
    return;
  }

  if (cur.attempts_done > config_.staging_max_retries)
  {
    fail_staging("robot '" + cur.name + "' did not reach its staging target within " +
                 std::to_string(config_.staging_max_retries) + " retr" +
                 (config_.staging_max_retries == 1 ? "y" : "ies"));
    return;
  }

  // Other PRESENT robots as static obstacles: already-processed robots
  // (index < current_index) are assumed to have arrived at their start
  // pose; not-yet-processed ones are at their (still current) measured
  // pose -- see StagingCore.h's OtherRobot doc comment.
  std::vector<staging::OtherRobot> others;
  others.reserve(staging_.robots.size() > 0 ? staging_.robots.size() - 1 : 0);
  for (size_t i = 0; i < staging_.robots.size(); ++i)
  {
    if (i == staging_.current_index)
      continue;
    staging::OtherRobot o;
    o.robot = staging_.robots[i].robot;
    o.current_pose =
        (i < staging_.current_index) ? staging_.robots[i].start_pose : staging_.robots[i].measured_pose;
    others.push_back(o);
  }

  const staging::StagingLegPlan plan = staging::plan_staging_leg(
      cur.robot, cur.measured_pose, cur.start_pose, others, staging_.scenario->params(),
      config_.staging_margin);
  if (!plan.success)
  {
    fail_staging("no staging path found for '" + cur.name + "': " + plan.failure_detail);
    return;
  }

  const ReloPush::trajectory traj = staging::build_staging_trajectory(cur.robot, plan.waypoints);
  const std::unordered_map<std::string, ReloPush::trajectory> overrides{{cur.name, traj}};
  const std::string suffix =
      "__staging_" + cur.name + "_a" + std::to_string(cur.attempts_done + 1);
  exec_mgr_.start_execution(staging_.scenario, {cur.name}, overrides, suffix);
  staging_.waiting_for_leg = true;
}

// ===========================================================================
// PART B: REAL-ROBOT MODE -- "move real robots to initial poses".
// ===========================================================================

std::optional<Pose> SimVizManager::stage_real_current_pose_of(const std::string &robot_name) const
{
  if (config_.stage_real_use_localization_source)
  {
    // TEST-ONLY path -- see SimVizConfig::stage_real_use_localization_
    // source's doc comment. Same freshness budget the sim STAGING PHASE's
    // "hardware mode" presence check uses.
    if (!real_pose_listener_.has_fresh_pose(robot_name, config_.staging_localization_wait_s))
      return std::nullopt;
    auto lp = real_pose_listener_.latest_pose(robot_name);
    return lp ? std::optional<Pose>(lp->pose) : std::nullopt;
  }

  // Production path: MocapManager::body_pose() alone does not freshness-
  // gate (see MocapCore.cpp) -- scan bodies() (which carries age_s) instead.
  for (const auto &b : mocap_.bodies())
  {
    if (b.published_name == robot_name && b.has_pose && b.age_s >= 0.0 &&
        b.age_s <= config_.mocap.freshness_window_s)
      return b.pose;
  }
  return std::nullopt;
}

bool SimVizManager::stage_real_robots(std::shared_ptr<ScenarioModel> scenario,
                                       std::string *err_out)
{
  if (staging_.active || stage_real_.active)
  {
    if (err_out)
      *err_out = "busy";
    return false;
  }
  if (!config_.real_mode)
  {
    if (err_out)
      *err_out = "stage_real_robots requires real_mode";
    return false;
  }
  if (exec_mgr_.is_busy())
  {
    if (err_out)
      *err_out = "busy";
    return false;
  }
  if (!scenario || !scenario->is_loaded())
  {
    if (err_out)
      *err_out = "no scenario";
    return false;
  }

  const auto robots = scenario->robots_sorted_by_name();
  if (robots.empty())
  {
    if (err_out)
      *err_out = "scenario has no robot entities";
    return false;
  }

  if (!config_.stage_real_use_localization_source && mocap_.state() != MocapState::Connected)
  {
    if (err_out)
      *err_out = "stage_real_robots requires MocapManager Connected";
    return false;
  }

  std::unordered_map<std::string, ReloPush::trajectory> traj_by_name;
  for (const auto &pr : scenario->robot_trajectories())
  {
    if (pr.first)
      traj_by_name[pr.first->name] = pr.second;
  }

  StageRealSequence seq;
  seq.scenario = scenario;

  // UNMAPPED ROBOTS ARE SIM-ONLY: a scenario robot with no "aliases" entry
  // pointing at it (robot_is_mapped(), MocapCore.h) is not physically
  // present -- it is skipped here entirely (never contributes to
  // missing_pose below) rather than blocking the whole move with an error,
  // since "move real robots to initial poses" only ever needs to move
  // robots that ARE present. A robot that IS mapped but has no fresh pose
  // (tracking lost) is still an error -- it is expected to be present and
  // reachable, unlike one that was simply never mapped.
  std::vector<std::string> missing_pose;
  std::vector<std::string> skipped_unmapped;
  for (EntityMeta *r : robots)
  {
    RobotMeta *robot_meta = dynamic_cast<RobotMeta *>(r);
    if (!robot_meta)
      continue; // defensive: robots_sorted_by_name() only ever returns EntityType::ROBOT entries.

    // The unmapped-robot skip below is a mocap-bridge-only concept: it
    // encodes "not mapped in Robot mapping -- not physically present" for
    // the production pose source. When the pluggable localization-source
    // path is active (config_.stage_real_use_localization_source), mocap
    // mappings are irrelevant to whether a robot is "present" -- every
    // scenario robot is expected to have a pose from that source, so none
    // are skipped here (matches pre-regression behavior for that path).
    if (!config_.stage_real_use_localization_source && !robot_is_mapped(mocap_.mapping(), r->name))
    {
      skipped_unmapped.push_back(r->name);
      continue;
    }

    auto pose = stage_real_current_pose_of(r->name);
    if (!pose)
    {
      missing_pose.push_back(r->name);
      continue;
    }

    StageRealRobotState st;
    st.name = r->name;
    st.robot = robot_meta;
    ReloPush::trajectory empty_traj;
    const auto traj_it = traj_by_name.find(r->name);
    st.target_pose =
        staging_start_pose_for(r, traj_it != traj_by_name.end() ? traj_it->second : empty_traj);
    st.measured_pose = *pose;
    seq.robots.push_back(std::move(st));
  }

  if (!skipped_unmapped.empty())
  {
    std::ostringstream oss;
    oss << "[mars_sim_viz] STAGE_REAL: robot(s) [";
    for (size_t i = 0; i < skipped_unmapped.size(); ++i)
      oss << (i ? ", " : "") << skipped_unmapped[i];
    oss << "] not mapped in Robot mapping -- treated as simulation-only, excluded from the move";
    std::cout << oss.str() << std::endl;
  }

  // Every MAPPED scenario robot must have a live pose here -- "move real
  // robots to initial poses" cannot silently skip a robot it believes is
  // physically present but cannot currently see (unlike an unmapped robot,
  // already excluded above).
  if (!missing_pose.empty())
  {
    std::ostringstream oss;
    oss << "stage_real_robots: no live pose for mapped robot(s): ";
    for (size_t i = 0; i < missing_pose.size(); ++i)
      oss << (i ? ", " : "") << missing_pose[i];
    if (err_out)
      *err_out = oss.str();
    return false;
  }

  if (seq.robots.empty())
  {
    if (err_out)
    {
      *err_out = skipped_unmapped.empty()
                     // Defensive only -- every robot failed the RobotMeta cast above, which
                     // cannot happen given how every scenario in this codebase constructs
                     // its ROBOT entities (mirrors start_execution_with_staging()'s own
                     // identical guard/reasoning).
                     ? "stage_real_robots: no robot resolved to a RobotMeta entity"
                     : "stage_real_robots: no scenario robot is mapped in Robot mapping -- "
                       "nothing to move (all treated as simulation-only)";
    }
    return false;
  }

  seq.active = true;
  stage_real_ = std::move(seq);
  return true;
}

std::string SimVizManager::stage_real_scenario_file(const std::string &path)
{
  if (exec_mgr_.is_busy() || staging_.active || stage_real_.active)
    return "ERR busy";

  std::shared_ptr<ScenarioModel> model;
  try
  {
    model = std::make_shared<ScenarioModel>(ScenarioModel::load_from_file(path));
  }
  catch (const std::exception &ex)
  {
    return std::string("ERR ") + ex.what();
  }

  std::string err;
  if (!stage_real_robots(model, &err))
    return "ERR " + (err.empty() ? std::string("busy") : err);

  // Same "remember this path for RESTART/bare STAGE_REAL" convention
  // execute_scenario_file() uses.
  note_scenario_path(path);

  return "ACK_STAGE_REAL";
}

// DESIGN part C: writes config_.real_robot_endpoints to config_.
// real_robots_config_path via the free function of the same name.
bool SimVizManager::save_real_robots_config(std::string *err) const
{
  return simviz::save_real_robots_config(config_.real_robot_endpoints,
                                          config_.real_robots_config_path, err);
}

// DESIGN part C: shared "STAGING_REAL <robot> (k/N)" text -- see this
// method's doc comment in SimVizCore.h. handle_command()'s STATUS branch
// below reuses this instead of duplicating the formatting.
std::string SimVizManager::stage_real_progress_text() const
{
  if (!stage_real_.active)
    return std::string();
  std::ostringstream oss;
  const size_t total = stage_real_.robots.size();
  const size_t idx = std::min(stage_real_.current_index, total == 0 ? 0 : total - 1);
  const std::string robot_name = (total == 0) ? std::string("?") : stage_real_.robots[idx].name;
  oss << robot_name << " (" << (idx + 1) << "/" << total << ")";
  return oss.str();
}

void SimVizManager::fail_stage_real(const std::string &reason)
{
  const std::string label = stage_real_.scenario ? stage_real_.scenario->label() : std::string();
  stage_real_ = StageRealSequence{};
  exec_mgr_.fail_externally(label, reason);
}

void SimVizManager::tick_stage_real()
{
  if (!stage_real_.active)
    return;

  if (stage_real_.waiting_for_leg)
  {
    // DEDICATED per-leg timeout (DESIGN: "traj duration + 15s"), checked
    // BEFORE consulting exec_mgr_'s own state so a leg against real
    // hardware that never converges gets Err'd out here rather than
    // silently riding ExecutionManager::tick()'s generic (and much
    // shorter/weaker) completion-forcing margin to a false "Done".
    if (stage_real_.leg_deadline_valid &&
        std::chrono::steady_clock::now() > stage_real_.leg_deadline)
    {
      const std::string &name = stage_real_.robots[stage_real_.current_index].name;
      fail_stage_real("stage_real leg for '" + name +
                       "' exceeded its bounded timeout (traj duration + " +
                       std::to_string(static_cast<int>(config_.stage_real_leg_timeout_margin_s)) +
                       "s) without reaching Done/Err");
      return;
    }

    const RunState st = exec_mgr_.state();
    if (st == RunState::Running)
      return; // leg still in flight.

    if (st == RunState::Err)
    {
      const std::string &name = stage_real_.robots[stage_real_.current_index].name;
      fail_stage_real("stage_real leg for '" + name +
                       "' failed to execute: " + exec_mgr_.error_reason());
      return;
    }

    if (st != RunState::Done)
      return; // Idle -- shouldn't happen mid-leg; wait rather than misbehave.

    // Leg finished -- re-query the pose source for this robot's ACTUAL
    // achieved pose (falls back to the target if the source lost it mid-
    // leg -- best-case assumption, matching tick_staging()'s own "capture
    // the achieved pose" step) before any further start_execution() call
    // (next leg) resets exec_mgr_'s OWN run-scoped localization listener --
    // irrelevant here since stage_real_current_pose_of() never reads that
    // one, but kept in the same place in the state machine for clarity.
    StageRealRobotState &cur = stage_real_.robots[stage_real_.current_index];
    auto live = stage_real_current_pose_of(cur.name);
    cur.measured_pose = live ? *live : cur.target_pose;
    cur.attempts_done++;
    stage_real_.waiting_for_leg = false;
    stage_real_.leg_deadline_valid = false; // no leg in flight -- nothing to time out.
    return; // re-decide (skip-now-verified / retry / fail) on the next tick.
  }

  if (stage_real_.current_index >= stage_real_.robots.size())
  {
    // Every robot processed -- done. Unlike the sim STAGING PHASE, there is
    // no main-run chain-in here: "move to initial poses" is the entire
    // action. Clear stage_real_ so a STATUS poll right after sees
    // exec_mgr_'s own (already Done, from the check above) state instead
    // of a stale "STAGING_REAL ..." string.
    stage_real_ = StageRealSequence{};
    return;
  }

  StageRealRobotState &cur = stage_real_.robots[stage_real_.current_index];
  const staging::Decision decision = staging::staging_decision(
      cur.measured_pose, cur.target_pose, config_.stage_real_pos_tol, config_.stage_real_yaw_tol);
  if (decision == staging::Decision::Skip)
  {
    stage_real_.current_index++;
    return;
  }

  if (cur.attempts_done > config_.staging_max_retries)
  {
    fail_stage_real("robot '" + cur.name + "' did not reach its staging target within " +
                     std::to_string(config_.staging_max_retries) + " retr" +
                     (config_.staging_max_retries == 1 ? "y" : "ies"));
    return;
  }

  // Other robots as static obstacles: already-processed robots (index <
  // current_index) are assumed to have arrived at their target pose;
  // not-yet-processed ones are at their (still current) measured pose --
  // mirrors tick_staging()'s own OtherRobot construction exactly.
  std::vector<staging::OtherRobot> others;
  others.reserve(stage_real_.robots.size() > 0 ? stage_real_.robots.size() - 1 : 0);
  for (size_t i = 0; i < stage_real_.robots.size(); ++i)
  {
    if (i == stage_real_.current_index)
      continue;
    staging::OtherRobot o;
    o.robot = stage_real_.robots[i].robot;
    o.current_pose = (i < stage_real_.current_index) ? stage_real_.robots[i].target_pose
                                                       : stage_real_.robots[i].measured_pose;
    others.push_back(o);
  }

  const staging::StagingLegPlan plan = staging::plan_staging_leg(
      cur.robot, cur.measured_pose, cur.target_pose, others, stage_real_.scenario->params(),
      config_.staging_margin);
  if (!plan.success)
  {
    fail_stage_real("no staging path found for '" + cur.name + "': " + plan.failure_detail);
    return;
  }

  const ReloPush::trajectory traj = staging::build_staging_trajectory(cur.robot, plan.waypoints);
  const std::unordered_map<std::string, ReloPush::trajectory> overrides{{cur.name, traj}};
  const std::string suffix =
      "__stage_real_" + cur.name + "_a" + std::to_string(cur.attempts_done + 1);
  // real_mode: real_mode is set on config_ (the whole session, via
  // --real-mode) -- exec_mgr_ was constructed with THIS config_, so
  // start_execution() below already spawns controller-only for this leg,
  // no separate flag needed here.
  exec_mgr_.start_execution(stage_real_.scenario, {cur.name}, overrides, suffix);
  stage_real_.waiting_for_leg = true;

  // DEDICATED per-leg timeout: traj duration + config_.
  // stage_real_leg_timeout_margin_s (default kStageRealLegTimeoutMarginS --
  // see this method's earlier doc comment / the constant's own comment /
  // SimVizConfig::stage_real_leg_timeout_margin_s's doc comment). traj.
  // CalcualteTimeStamps() (inside build_staging_trajectory(), above) starts
  // every leg at t=0, so the last waypoint's time IS the leg's planned
  // duration.
  const double leg_duration_s =
      (traj.trajectory_points && !traj.trajectory_points->empty())
          ? static_cast<double>(traj.trajectory_points->back().time)
          : 0.0;
  stage_real_.leg_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(
          std::max(0.0, leg_duration_s) + config_.stage_real_leg_timeout_margin_s));
  stage_real_.leg_deadline_valid = true;
}

} // namespace simviz
