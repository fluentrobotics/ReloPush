#include "MocapCore.h"

#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFileInfo>
#include <QProcessEnvironment>
#include <QStringList>

#include <zmq.hpp>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace simviz
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kSubRecvTimeoutMs = 200;
constexpr int kTerminateGraceMs = 1500;
constexpr int kStartGraceMs = 2000;

// Wraps `angle` into (-pi, pi] -- field-for-field mirror of
// MPC/include/mpc/OptiTrackCore.h's normalize_angle(), same
// convention used by apply_transform_matrix()/apply_full_transform_chain()
// below.
double normalize_angle(double angle)
{
  while (angle <= -kPi)
    angle += 2.0 * kPi;
  while (angle > kPi)
    angle -= 2.0 * kPi;
  return angle;
}

// Spawns `binary_path args...` wrapped in "env -i HOME=.. USER=.. PATH=..
// QT_QPA_PLATFORM=offscreen" -- identical convention to SimVizCore.cpp's
// (anonymous-namespace, so independently duplicated here) spawn_wrapped_
// process(), except MergedChannels (this class needs to parse the bridge's
// own stdout for the Motive-assets inventory line -- SimVizCore.cpp's
// ExecutionManager never reads its children's output at all, hence
// ForwardedChannels there vs MergedChannels+captured here).
QProcess *spawn_wrapped_bridge(QObject *parent, const std::string &binary_path,
                                const QStringList &args)
{
  auto *proc = new QProcess(parent);
  proc->setProcessChannelMode(QProcess::MergedChannels);

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

// ===========================================================================
// Pure helpers: transform parsing.
// ===========================================================================

TransformInfo parse_transform_info(const nlohmann::json &map_config)
{
  TransformInfo info;

  if (map_config.contains("y_up") && map_config["y_up"].is_boolean())
    info.y_up = map_config["y_up"].get<bool>();

  bool has_matrix_key =
      map_config.contains("mocap_to_world_matrix") && map_config["mocap_to_world_matrix"].is_array();

  if (has_matrix_key)
  {
    const nlohmann::json &rows_j = map_config["mocap_to_world_matrix"];
    bool shape_ok = rows_j.is_array() && rows_j.size() == 3;
    std::vector<std::vector<double>> rows;
    if (shape_ok)
    {
      for (const auto &row_j : rows_j)
      {
        if (!row_j.is_array() || row_j.size() != 3)
        {
          shape_ok = false;
          break;
        }
        std::vector<double> row;
        for (const auto &v : row_j)
        {
          if (!v.is_number())
          {
            shape_ok = false;
            break;
          }
          row.push_back(v.get<double>());
        }
        if (!shape_ok)
          break;
        rows.push_back(row);
      }
    }

    if (!shape_ok)
    {
      info.parse_error = true;
      info.error = "mocap_to_world_matrix must be a 3x3 array of numbers";
      // info.matrix stays identity default.
    }
    else if (std::fabs(rows[2][0]) > 1e-6 || std::fabs(rows[2][1]) > 1e-6 ||
             std::fabs(rows[2][2] - 1.0) > 1e-6)
    {
      info.parse_error = true;
      info.error = "mocap_to_world_matrix bottom row must be [0,0,1]";
    }
    else
    {
      info.matrix.a = rows[0][0];
      info.matrix.b = rows[0][1];
      info.matrix.tx = rows[0][2];
      info.matrix.c = rows[1][0];
      info.matrix.d = rows[1][1];
      info.matrix.ty = rows[1][2];
      info.matrix_from_legacy = false;

      const double a = info.matrix.a, b = info.matrix.b, c = info.matrix.c, d = info.matrix.d;
      const double col0_norm = std::sqrt(a * a + c * c);
      const double col1_norm = std::sqrt(b * b + d * d);
      const double dot = a * b + c * d;
      const double det = a * d - b * c;
      const double tol = 1e-3;
      if (std::fabs(col0_norm - 1.0) > tol || std::fabs(col1_norm - 1.0) > tol ||
          std::fabs(dot) > tol || std::fabs(det - 1.0) > tol)
      {
        info.non_rigid_warning = true;
        info.warning = "mocap_to_world_matrix rotation block is not a pure rotation "
                        "(scale/shear present) -- yaw derivation is only approximate";
      }
    }
  }
  else
  {
    double x0 = map_config.value("x0", 0.0);
    double y0 = map_config.value("y0", 0.0);
    double theta0 = map_config.value("theta0", 0.0);
    info.legacy_x0 = x0;
    info.legacy_y0 = y0;
    info.legacy_theta0 = theta0;
    info.matrix_from_legacy = true;
    info.matrix.a = std::cos(theta0);
    info.matrix.b = -std::sin(theta0);
    info.matrix.tx = x0;
    info.matrix.c = std::sin(theta0);
    info.matrix.d = std::cos(theta0);
    info.matrix.ty = y0;
  }

  info.rotation_deg = std::atan2(info.matrix.c, info.matrix.a) * 180.0 / kPi;

  if (map_config.contains("yaw_offset") && map_config["yaw_offset"].is_object())
  {
    for (auto it = map_config["yaw_offset"].begin(); it != map_config["yaw_offset"].end(); ++it)
    {
      if (it.value().is_number())
        info.yaw_offsets[it.key()] = it.value().get<double>();
    }
  }

  return info;
}

Pose apply_transform_matrix(const AffineTransform2D &matrix, double x, double y, double yaw)
{
  const double xw = matrix.a * x + matrix.b * y + matrix.tx;
  const double yw = matrix.c * x + matrix.d * y + matrix.ty;
  const double yaw_w = normalize_angle(std::atan2(matrix.c, matrix.a) + yaw);
  return Pose(xw, yw, yaw_w);
}

Pose apply_full_transform_chain(const TransformInfo &info, double raw_x, double raw_y,
                                 double raw_yaw, const std::string &robot_name)
{
  Pose world = apply_transform_matrix(info.matrix, raw_x, raw_y, raw_yaw);
  double yaw_offset = 0.0;
  auto it = info.yaw_offsets.find(robot_name);
  if (it != info.yaw_offsets.end())
    yaw_offset = it->second;
  world.yaw = normalize_angle(world.yaw + yaw_offset);
  return world;
}

Pose mocap_rotation(double x_m, double /*y_m*/, double z_m, double qx, double qy, double qz,
                     double qw)
{
  // Field-for-field mirror of MPC/include/mpc/OptiTrackCore.h's
  // mocap_rotation() -- see this file's own header comment for why this
  // exists despite having no live caller (no raw quaternion ever reaches
  // this process). vx/vy/vz = R(q)*(1,0,0)^T, the SAME first-column
  // formula OptiTrackCore.h's quat_to_planar_yaw() derives from (Hamilton,
  // active, right-handed qx,qy,qz,qw).
  const double vx = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double vz = 2.0 * (qx * qz - qy * qw);
  return Pose(x_m, -z_m, normalize_angle(std::atan2(-vz, vx)));
}

StagedTransformView compute_staged_transform_view(const TransformInfo &info,
                                                    const Pose &final_pose,
                                                    const std::string &robot_name)
{
  StagedTransformView v;
  v.final_x = final_pose.x;
  v.final_y = final_pose.y;
  v.final_yaw = final_pose.yaw;

  double yaw_offset = 0.0;
  auto it = info.yaw_offsets.find(robot_name);
  if (it != info.yaw_offsets.end())
    yaw_offset = it->second;

  // Undo yaw_offset (position is untouched by it -- see
  // apply_full_transform_chain()'s own body).
  const double post_calib_yaw = normalize_angle(final_pose.yaw - yaw_offset);

  // Undo the calibration matrix: [xw,yw]=[[a,b],[c,d]]*[x,y]+[tx,ty], so
  // [x,y] = [[a,b],[c,d]]^-1 * ([xw,yw]-[tx,ty]).
  const AffineTransform2D &m = info.matrix;
  const double det = m.a * m.d - m.b * m.c;
  if (std::fabs(det) < 1e-9)
  {
    v.ok = false;
    v.error = "calibration matrix is singular (det~=0) -- cannot invert to reconstruct the "
              "raw/mocap-rotation stages";
    return v;
  }
  const double dx = final_pose.x - m.tx;
  const double dy = final_pose.y - m.ty;
  v.mocap_x = (m.d * dx - m.b * dy) / det;
  v.mocap_y = (-m.c * dx + m.a * dy) / det;
  v.mocap_yaw = normalize_angle(post_calib_yaw - std::atan2(m.c, m.a));

  // Undo mocap_rotation()'s position mapping, conditioned on info.y_up --
  // yaw is untouched (heading-invariant, see mocap_rotation()'s doc
  // comment): raw_yaw == mocap_yaw always.
  v.raw_x_m = v.mocap_x;
  v.raw_yaw = v.mocap_yaw;
  if (info.y_up)
  {
    v.raw_z_m = -v.mocap_y;
    v.raw_y_m = 0.0;
    v.raw_height_unknown_axis = "y";
  }
  else
  {
    v.raw_y_m = v.mocap_y;
    v.raw_z_m = 0.0;
    v.raw_height_unknown_axis = "z";
  }

  v.ok = true;
  return v;
}

nlohmann::json set_calibration_in_mapping(nlohmann::json config, double x0, double y0,
                                           double theta0, bool y_up)
{
  config["x0"] = x0;
  config["y0"] = y0;
  config["theta0"] = theta0;
  config["y_up"] = y_up;
  if (config.contains("mocap_to_world_matrix"))
    config.erase("mocap_to_world_matrix");
  return config;
}

nlohmann::json set_yaw_offset_in_mapping(nlohmann::json config, const std::string &robot_name,
                                          double yaw_offset_rad)
{
  if (!config.contains("yaw_offset") || !config["yaw_offset"].is_object())
    config["yaw_offset"] = nlohmann::json::object();
  config["yaw_offset"][robot_name] = yaw_offset_rad;
  return config;
}

// ===========================================================================
// Mapping model.
// ===========================================================================

nlohmann::json load_mapping_json(const std::string &path)
{
  std::ifstream f(path);
  if (!f.is_open())
    throw std::runtime_error("cannot open mapping config: " + path);
  nlohmann::json j;
  f >> j; // propagates nlohmann::json::parse_error on invalid JSON.
  return j;
}

nlohmann::json set_alias_in_mapping(nlohmann::json config, const std::string &motive_name,
                                     const std::string &planner_name)
{
  if (!config.contains("aliases") || !config["aliases"].is_object())
    config["aliases"] = nlohmann::json::object();
  config["aliases"][motive_name] = planner_name;
  return config;
}

bool save_mapping_json(const nlohmann::json &config, const std::string &path, std::string *err)
{
  const QFileInfo info(QString::fromStdString(path));
  const std::string abs_path = info.absoluteFilePath().toStdString();

  // Create the parent directory if it's missing -- a fresh checkout/rebuild
  // dir may not have MPC/config yet, and failing here with just
  // ofstream's generic "won't open" (no errno, no absolute path) is exactly
  // the unhelpful "fails to save" report this function's callers exist to
  // fix (see MocapManager::save_mapping()'s doc comment).
  const QDir parent_dir = info.dir();
  if (!parent_dir.exists() && !parent_dir.mkpath("."))
  {
    if (err)
      *err = "cannot create directory '" + parent_dir.absolutePath().toStdString() +
             "' for '" + abs_path + "'";
    return false;
  }

  std::ofstream f(path);
  if (!f.is_open())
  {
    if (err)
      *err = "cannot open '" + abs_path + "' for write: " + std::strerror(errno);
    return false;
  }
  f << config.dump(2);
  if (!f.good())
  {
    if (err)
      *err = "write failed: '" + abs_path + "': " + std::strerror(errno);
    return false;
  }
  return true;
}

// ===========================================================================
// Motive asset inventory line parsing.
// ===========================================================================

std::vector<MotiveAssetEntry> parse_motive_assets_line(const std::string &line)
{
  std::vector<MotiveAssetEntry> out;
  const std::string marker = "Motive assets:";
  const auto marker_pos = line.find(marker);
  if (marker_pos == std::string::npos)
    return out;

  const std::string id_key = "rigidbody id=";
  size_t search_from = marker_pos + marker.size();
  while (true)
  {
    const auto id_pos = line.find(id_key, search_from);
    if (id_pos == std::string::npos)
      break;
    size_t p = id_pos + id_key.size();
    size_t id_end = p;
    while (id_end < line.size() && (std::isdigit(static_cast<unsigned char>(line[id_end])) ||
                                     line[id_end] == '-'))
      ++id_end;
    if (id_end == p)
    {
      search_from = p;
      continue;
    }
    int id = 0;
    try
    {
      id = std::stoi(line.substr(p, id_end - p));
    }
    catch (...)
    {
      search_from = id_end;
      continue;
    }

    const auto name_key_pos = line.find("name='", id_end);
    if (name_key_pos == std::string::npos)
      break;
    const size_t name_start = name_key_pos + 6;
    const auto name_end = line.find('\'', name_start);
    if (name_end == std::string::npos)
      break;
    const std::string name = line.substr(name_start, name_end - name_start);

    const auto parent_key_pos = line.find("parent=", name_end);
    if (parent_key_pos == std::string::npos)
      break;
    size_t pp = parent_key_pos + 7;
    size_t parent_end = pp;
    while (parent_end < line.size() && (std::isdigit(static_cast<unsigned char>(line[parent_end])) ||
                                         line[parent_end] == '-'))
      ++parent_end;
    int parent_id = -1;
    if (parent_end > pp)
    {
      try
      {
        parent_id = std::stoi(line.substr(pp, parent_end - pp));
      }
      catch (...)
      {
        parent_id = -1;
      }
    }

    MotiveAssetEntry e;
    e.id = id;
    e.name = name;
    e.parent_id = parent_id;
    out.push_back(e);
    search_from = parent_end;
  }
  return out;
}

std::string expected_published_name(const std::string &motive_name, const nlohmann::json &mapping)
{
  if (mapping.contains("aliases") && mapping["aliases"].is_object())
  {
    auto it = mapping["aliases"].find(motive_name);
    if (it != mapping["aliases"].end() && it.value().is_string())
      return it.value().get<std::string>();
  }
  if (motive_name.empty())
    return motive_name; // bridge would use a "body<id>" fallback instead -- this pure function
                         // has no id to reconstruct that with; callers keyed on id should special
                         // -case an empty name themselves (MotiveAssetEntry always carries an id).
  std::string sanitized = motive_name;
  for (char &c : sanitized)
  {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '-';
    if (!ok)
      c = '_';
  }
  return sanitized;
}

bool robot_is_mapped(const nlohmann::json &mapping, const std::string &planner_name)
{
  if (!mapping.contains("aliases") || !mapping["aliases"].is_object())
    return false;
  for (auto it = mapping["aliases"].begin(); it != mapping["aliases"].end(); ++it)
  {
    if (it.value().is_string() && it.value().get<std::string>() == planner_name)
      return true;
  }
  return false;
}

// ===========================================================================
// MocapManager
// ===========================================================================

const char *mocap_state_name(MocapState state)
{
  switch (state)
  {
  case MocapState::Disconnected:
    return "DISCONNECTED";
  case MocapState::Connecting:
    return "CONNECTING";
  case MocapState::Connected:
    return "CONNECTED";
  case MocapState::Error:
    return "ERROR";
  }
  return "UNKNOWN";
}

MocapManager::MocapManager(MocapConfig config, QObject *parent)
    : QObject(parent), config_(std::move(config))
{
  // Resolve ONCE, up front -- see resolve_map_config_path()'s doc comment in
  // MocapCore.h for why an empty caller-supplied value can no longer mean
  // "leave it empty" for THIS field (unlike mocap_config_path/bridge_path,
  // this process itself loads/saves it, not just the spawned bridge).
  // config_.map_config_path is never empty after this line.
  config_.map_config_path = resolve_map_config_path();
  try
  {
    mapping_json_ = load_mapping_json(config_.map_config_path);
  }
  catch (const std::exception &ex)
  {
    mapping_load_error_ = ex.what();
    mapping_json_ = nlohmann::json::object();
  }
}

MocapManager::~MocapManager()
{
  stop_bridge();
}

std::string MocapManager::resolve_bridge_path() const
{
  if (!config_.bridge_path.empty())
    return config_.bridge_path;
  const QString app_dir = QCoreApplication::applicationDirPath();
  const QString candidate = app_dir + "/../MPC/optitrack_zmq_bridge";
  return QFileInfo(candidate).absoluteFilePath().toStdString();
}

// See this method's doc comment in MocapCore.h.
std::string MocapManager::resolve_map_config_path() const
{
  if (!config_.map_config_path.empty())
    return QFileInfo(QString::fromStdString(config_.map_config_path)).absoluteFilePath().toStdString();
  const QString app_dir = QCoreApplication::applicationDirPath();
  const QString candidate = app_dir + "/../../MPC/config/mocap_map_config.json";
  return QFileInfo(candidate).absoluteFilePath().toStdString();
}

bool MocapManager::start_bridge(std::string *err)
{
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ == MocapState::Connecting || state_ == MocapState::Connected)
    {
      if (err)
        *err = "already connected/connecting";
      return false;
    }
  }

  const std::string bridge_path = resolve_bridge_path();
  {
    std::ifstream probe(bridge_path);
    if (!probe.good())
    {
      if (err)
        *err = "bridge binary not found: " + bridge_path;
      return false;
    }
  }

  QStringList args;
  // Deliberately NO --robots/--rigid-body-ids/--rigid-body-names -- this is
  // what selects the bridge's AUTO-DISCOVERY mode (see this file's header
  // comment).
  if (!config_.mocap_config_path.empty())
    args << "--mocap-config" << QString::fromStdString(config_.mocap_config_path);
  if (!config_.map_config_path.empty())
    args << "--map-config" << QString::fromStdString(config_.map_config_path);
  if (config_.server_ip_explicit)
    args << "--server-ip" << QString::fromStdString(config_.server_ip);
  if (config_.mode_explicit)
    args << "--mode" << QString::fromStdString(config_.mode);
  if (config_.command_port_explicit)
    args << "--command-port" << QString::number(config_.command_port);
  if (config_.local_data_port_explicit)
    args << "--local-data-port" << QString::number(config_.local_data_port);
  args << "--loc-port-start" << QString::number(config_.loc_port_start);
  args << "--publish-rate" << QString::number(config_.publish_rate_hz);

  QProcess *proc = spawn_wrapped_bridge(this, bridge_path, args);
  if (!proc->waitForStarted(kStartGraceMs))
  {
    if (err)
      *err = "bridge failed to start: " + proc->errorString().toStdString();
    proc->deleteLater();
    return false;
  }

  proc_ = proc;
  stdout_buffer_.clear();
  bridge_should_be_running_ = true;

  {
    std::lock_guard<std::mutex> lk(inventory_mutex_);
    motive_assets_.clear();
  }

  sub_start_time_ = std::chrono::steady_clock::now();
  sub_start_time_valid_ = true;
  sub_stop_requested_.store(false);
  sub_thread_ = std::thread(&MocapManager::sub_thread_main, this, config_.loc_port_start);

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    state_ = MocapState::Connecting;
    error_reason_.clear();
  }
  return true;
}

void MocapManager::stop_bridge()
{
  bridge_should_be_running_ = false;

  if (proc_)
  {
    proc_->terminate();
    if (!proc_->waitForFinished(kTerminateGraceMs))
    {
      proc_->kill();
      proc_->waitForFinished(500);
    }
    proc_->deleteLater();
    proc_ = nullptr;
  }

  sub_stop_requested_.store(true);
  if (sub_thread_.joinable())
    sub_thread_.join();
  sub_stop_requested_.store(false);

  std::lock_guard<std::mutex> lk(state_mutex_);
  state_ = MocapState::Disconnected;
  error_reason_.clear();
}

void MocapManager::set_error(const std::string &reason)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  state_ = MocapState::Error;
  error_reason_ = reason;
}

void MocapManager::drain_stdout()
{
  if (!proc_)
    return;
  const QByteArray chunk = proc_->readAllStandardOutput();
  if (chunk.isEmpty())
    return;
  stdout_buffer_.append(chunk.constData(), static_cast<size_t>(chunk.size()));

  size_t pos;
  while ((pos = stdout_buffer_.find('\n')) != std::string::npos)
  {
    std::string line = stdout_buffer_.substr(0, pos);
    stdout_buffer_.erase(0, pos + 1);
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty())
      continue;

    std::cout << "[MOCAP] " << line << std::endl;

    std::vector<MotiveAssetEntry> parsed = parse_motive_assets_line(line);
    if (line.find("Motive assets:") != std::string::npos)
    {
      std::lock_guard<std::mutex> lk(inventory_mutex_);
      motive_assets_ = std::move(parsed);
    }
  }
}

void MocapManager::tick()
{
  if (!bridge_should_be_running_)
    return;

  // MergedChannels stdout capture (drain_stdout() below) and prompt
  // NotRunning/crash detection both rely on QProcess's internal
  // QSocketNotifier-driven buffering, which is only serviced when the Qt
  // event loop is pumped -- callers of this class (SimVizManager::tick(),
  // this file's own tests) poll tick() directly without ever calling
  // QCoreApplication::exec(), so a zero-timeout processEvents() here is
  // what actually transfers pending child-process I/O/exit notifications
  // into Qt's buffers before this method reads them below (mirrors the
  // same "no long-running exec(), just poll" convention every caller of
  // this class already uses).
  QCoreApplication::processEvents(QEventLoop::AllEvents, 0);

  drain_stdout();

  if (proc_ && proc_->state() == QProcess::NotRunning)
  {
    const int exit_code = proc_->exitCode();
    const QProcess::ExitStatus exit_status = proc_->exitStatus();
    proc_->deleteLater();
    proc_ = nullptr;
    bridge_should_be_running_ = false;

    sub_stop_requested_.store(true);
    if (sub_thread_.joinable())
      sub_thread_.join();
    sub_stop_requested_.store(false);

    std::ostringstream oss;
    oss << "bridge exited unexpectedly (code=" << exit_code
        << ", crashed=" << (exit_status == QProcess::CrashExit ? "yes" : "no") << ")";
    set_error(oss.str());
    return;
  }

  // Re-evaluate the Connecting<->Connected freshness gate (both
  // directions -- see tick()'s header-comment doc).
  bool any_fresh = false;
  if (sub_start_time_valid_)
  {
    const double now_rel =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - sub_start_time_).count();
    std::lock_guard<std::mutex> lk(bodies_mutex_);
    for (const auto &kv : bodies_)
    {
      if (!kv.second.has_pose)
        continue;
      const double age = now_rel - kv.second.last_update_steady_s;
      if (age <= config_.freshness_window_s)
      {
        any_fresh = true;
        break;
      }
    }
  }

  std::lock_guard<std::mutex> lk(state_mutex_);
  if (state_ == MocapState::Connecting && any_fresh)
    state_ = MocapState::Connected;
  else if (state_ == MocapState::Connected && !any_fresh)
    state_ = MocapState::Connecting;
}

MocapState MocapManager::state() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return state_;
}

std::string MocapManager::error_reason() const
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  return error_reason_;
}

qint64 MocapManager::process_pid() const
{
  return proc_ ? proc_->processId() : 0;
}

std::vector<MocapManager::Body> MocapManager::bodies() const
{
  std::vector<Body> out;
  const double now_rel = sub_start_time_valid_
                              ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                                sub_start_time_)
                                    .count()
                              : -1.0;
  std::lock_guard<std::mutex> lk(bodies_mutex_);
  out.reserve(bodies_.size());
  for (const auto &kv : bodies_)
  {
    Body b;
    b.published_name = kv.first;
    b.pose = kv.second.pose;
    b.has_pose = kv.second.has_pose;
    b.age_s = (b.has_pose && sub_start_time_valid_) ? (now_rel - kv.second.last_update_steady_s)
                                                      : -1.0;
    out.push_back(std::move(b));
  }
  return out;
}

std::optional<Pose> MocapManager::body_pose(const std::string &published_name) const
{
  std::lock_guard<std::mutex> lk(bodies_mutex_);
  auto it = bodies_.find(published_name);
  if (it == bodies_.end() || !it->second.has_pose)
    return std::nullopt;
  return it->second.pose;
}

std::optional<MocapManager::TimedPose>
MocapManager::latest_timed_pose(const std::string &published_name) const
{
  std::lock_guard<std::mutex> lk(bodies_mutex_);
  auto it = bodies_.find(published_name);
  if (it == bodies_.end() || !it->second.has_pose)
    return std::nullopt;
  TimedPose out;
  out.t = it->second.last_update_steady_s;
  out.pose = it->second.pose;
  return out;
}

std::vector<MotiveAssetEntry> MocapManager::motive_assets() const
{
  std::lock_guard<std::mutex> lk(inventory_mutex_);
  return motive_assets_;
}

void MocapManager::set_alias(const std::string &motive_name, const std::string &planner_name)
{
  mapping_json_ = set_alias_in_mapping(std::move(mapping_json_), motive_name, planner_name);
}

void MocapManager::set_calibration(double x0, double y0, double theta0, bool y_up)
{
  mapping_json_ = set_calibration_in_mapping(std::move(mapping_json_), x0, y0, theta0, y_up);
}

void MocapManager::set_yaw_offset(const std::string &robot_name, double yaw_offset_rad)
{
  mapping_json_ = set_yaw_offset_in_mapping(std::move(mapping_json_), robot_name, yaw_offset_rad);
}

bool MocapManager::save_mapping(std::string *err)
{
  // config_.map_config_path is resolved to a non-empty absolute path once,
  // in the constructor (see resolve_map_config_path()) -- this guard is a
  // defensive no-op in practice now, kept only so a future caller that
  // somehow reset the field back to empty still gets a clear error instead
  // of save_mapping_json()'s "cannot create directory ''" message.
  if (config_.map_config_path.empty())
  {
    if (err)
      *err = "no map_config_path configured";
    return false;
  }
  return save_mapping_json(mapping_json_, config_.map_config_path, err);
}

bool MocapManager::reload_mapping(std::string *err)
{
  // Same load logic as the constructor (config_.map_config_path is already
  // resolved to a non-empty absolute path by then -- see
  // resolve_map_config_path()) -- see this method's doc comment in
  // MocapCore.h for why it exists as a separate, later-callable method.
  try
  {
    mapping_json_ = load_mapping_json(config_.map_config_path);
    mapping_load_error_.clear();
    return true;
  }
  catch (const std::exception &ex)
  {
    mapping_load_error_ = ex.what();
    if (err)
      *err = ex.what();
    return false;
  }
}

void MocapManager::sub_thread_main(int port)
{
  try
  {
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::sub);
    sock.set(zmq::sockopt::linger, 0);
    sock.set(zmq::sockopt::rcvtimeo, kSubRecvTimeoutMs);
    const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(port);
    sock.connect(endpoint);
    // Subscribe to EVERY topic -- auto-discovery mode publishes one topic
    // per body ("/<published_name>/localization") on this single port, and
    // this class does not know the body names in advance.
    sock.set(zmq::sockopt::subscribe, "");

    while (!sub_stop_requested_.load())
    {
      while (true)
      {
        zmq::message_t topic_msg;
        auto res = sock.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!res)
          break;
        std::string topic(static_cast<const char *>(topic_msg.data()), topic_msg.size());
        std::string payload;
        if (topic_msg.more())
        {
          zmq::message_t payload_msg;
          if (sock.recv(payload_msg, zmq::recv_flags::none))
            payload.assign(static_cast<const char *>(payload_msg.data()), payload_msg.size());
        }
        if (payload.empty())
          continue;

        // Expect "/<name>/localization".
        const std::string suffix = "/localization";
        if (topic.size() <= suffix.size() + 1 || topic.front() != '/' ||
            topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0)
          continue;
        const std::string name = topic.substr(1, topic.size() - 1 - suffix.size());
        if (name.empty())
          continue;

        double x = 0.0, y = 0.0, yaw = 0.0;
        try
        {
          nlohmann::json j = nlohmann::json::parse(payload);
          if (!j.contains("x") || !j.contains("y") || !j.contains("yaw"))
            continue;
          x = j.at("x").get<double>();
          y = j.at("y").get<double>();
          yaw = j.at("yaw").get<double>();
        }
        catch (const std::exception &)
        {
          continue;
        }

        const double now_rel =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - sub_start_time_)
                .count();

        BodySample sample;
        sample.pose = Pose(x, y, yaw);
        sample.has_pose = true;
        sample.last_update_steady_s = now_rel;

        std::lock_guard<std::mutex> lk(bodies_mutex_);
        bodies_[name] = sample;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  catch (const std::exception &)
  {
    // Background diagnostics thread -- swallow errors, just stop updating
    // (mirrors LocalizationListener::thread_main()'s convention).
  }
}

} // namespace simviz
