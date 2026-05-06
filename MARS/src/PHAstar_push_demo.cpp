/*****************************************************************
 * Prioritized Hybrid Astar Demo with Pushing Tasks from ReloPush
 * Refactored for Debuggability & Modularity
 *
 * 2025.11.9
 * Refactored Version
 ******************************************************************/

#include <LoadFinalSequence.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/Reeds_Shepp.h>
#include <Task.h>
#include <PHAstar/Visualization.h>
#include <PHAstar/CollisionUtils.h>
#include <PHAstarPushDemoOptions.h>
#include <PHAstarPushDemoTypes.h>
#include <ReloPush/FinalSequenceHandoff.h>
#include <ReloPush/config.h>
#include <ReloPush/TaskAllocation.hpp>
#include <ReloPush/PrintInColor.hpp>
#include <QFont>
#include <QImage>
#include <QPainterPath>
#include <QPen>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

// for finding parking
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <limits>
#include <mutex>
#include <numeric>
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>

bool DEBUG_VIS = false;

std::string default_sequence_path();
std::string sanitize_filename_component(const std::string &label);
double shared_collision_check_step(const Params &params);
Params initialize_params(const std::vector<FinalAllocation> &loadedSequence,
                         const RuntimeOptions &options);

ReferenceExperienceGraphOptions reference_egraph_options_from_runtime(
    const RuntimeOptions &options);

namespace
{
  thread_local std::mt19937 g_parking_rng;
  thread_local bool g_parking_rng_initialized = false;
  thread_local std::uint32_t g_parking_rng_seed = 0;
  thread_local ParkingCandidateMode g_parking_candidate_mode = ParkingCandidateMode::EXPAND;
  thread_local std::unordered_map<std::string, double> g_recent_failed_relocations;

  const char *parking_candidate_mode_name(ParkingCandidateMode mode)
  {
    switch (mode)
    {
    case ParkingCandidateMode::REVERSE_RECENT:
      return "reverse-recent-path";
    case ParkingCandidateMode::REVERSE_RECENT_SHORTER:
      return "reverse-recent-shorter";
    case ParkingCandidateMode::CONNECTED_VFH:
      return "connected-vfh";
    case ParkingCandidateMode::CONNECTED:
      return "connected-primitives";
    case ParkingCandidateMode::EXPAND:
      return "expand-primitives";
    case ParkingCandidateMode::RANDOM:
    default:
      return "randomized-candidates";
    }
  }

  const char *parking_candidate_mode_name()
  {
    return parking_candidate_mode_name(g_parking_candidate_mode);
  }

  void initialize_parking_rng(bool has_fixed_seed, std::uint32_t fixed_seed)
  {
    if (has_fixed_seed)
    {
      g_parking_rng_seed = fixed_seed;
    }
    else
    {
      std::random_device rd;
      g_parking_rng_seed = rd();
    }

    g_parking_rng.seed(g_parking_rng_seed);
    g_parking_rng_initialized = true;
  }

  std::mt19937 &parking_rng()
  {
    if (!g_parking_rng_initialized)
    {
      initialize_parking_rng(false, 0);
    }
    return g_parking_rng;
  }

  std::uint32_t parking_rng_seed()
  {
    if (!g_parking_rng_initialized)
    {
      initialize_parking_rng(false, 0);
    }
    return g_parking_rng_seed;
  }

  std::unordered_map<std::string, double> &recent_failed_relocation_cache()
  {
    return g_recent_failed_relocations;
  }

  void reset_thread_local_planning_state()
  {
    g_recent_failed_relocations.clear();
  }
}

std::string csv_escape(const std::string &value)
{
  bool need_quotes = value.find(',') != std::string::npos ||
                     value.find('"') != std::string::npos ||
                     value.find('\n') != std::string::npos;
  if (!need_quotes)
    return value;

  std::string escaped = "\"";
  for (char c : value)
  {
    if (c == '"')
      escaped += "\"\"";
    else
      escaped += c;
  }
  escaped += "\"";
  return escaped;
}

const char *planning_status_name(PlanningStatus status)
{
  switch (status)
  {
  case PlanningStatus::SUCCESS:
    return "SUCCESS";
  case PlanningStatus::START_INVALID_COLLISION:
    return "START_INVALID_COLLISION";
  case PlanningStatus::START_OUT_OF_BOUNDS:
    return "START_OUT_OF_BOUNDS";
  case PlanningStatus::GOAL_INVALID_COLLISION:
    return "GOAL_INVALID_COLLISION";
  case PlanningStatus::GOAL_OUT_OF_BOUNDS:
    return "GOAL_OUT_OF_BOUNDS";
  case PlanningStatus::NO_PATH_FOUND:
    return "NO_PATH_FOUND";
  case PlanningStatus::TIMEOUT_EXCEEDED:
    return "TIMEOUT_EXCEEDED";
  case PlanningStatus::HIGH_COST_UNFEASIBLE:
    return "HIGH_COST_UNFEASIBLE";
  case PlanningStatus::BLOCKED_BY_ROBOT:
    return "BLOCKED_BY_ROBOT";
  case PlanningStatus::INTERNAL_ERROR:
  default:
    return "INTERNAL_ERROR";
  }
}

void write_task_csv_log(const std::string &csv_path,
                        const std::vector<TaskCsvRow> &rows)
{
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open CSV file: " << csv_path << std::endl;
    return;
  }

  ofs << "task_id,object,status,robot,start_time,end_time,total_waiting,attempts,failure_reason\n";
  ofs << std::fixed << std::setprecision(2);

  for (const auto &row : rows)
  {
    ofs << row.task_id << ","
        << csv_escape(row.object_name) << ","
        << csv_escape(row.status) << ","
        << csv_escape(row.robot_name) << ","
        << row.start_time << ","
        << row.end_time << ","
        << row.total_waiting << ","
        << row.attempts << ","
        << csv_escape(row.failure_reason) << "\n";
  }

  std::cout << "[Log] Wrote task execution CSV: " << csv_path << std::endl;
}

struct OrientationJumpRecord
{
  std::string scenario_label;
  std::string source;
  std::string robot_name;
  double prev_time = 0.0;
  double curr_time = 0.0;
  Pose prev_pose;
  Pose curr_pose;
  double raw_yaw_delta = 0.0;
  double wrapped_yaw_delta = 0.0;
  double distance = 0.0;
};

void write_timetable_robot_pose_csv(const std::string &csv_path,
                                    const std::string &scenario_label,
                                    const TimeTable &timetable,
                                    const std::unordered_map<std::string, EntityMeta *> &entities,
                                    double sample_step)
{
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open timetable pose CSV: "
              << csv_path << std::endl;
    return;
  }

  ofs << "scenario,robot,time,x,y,yaw,yaw_unwrapped\n";
  ofs << std::fixed << std::setprecision(6);

  const double max_t = timetable.get_max_time();
  for (const auto &[name, entity] : entities)
  {
    if (!entity || entity->type != EntityType::ROBOT)
      continue;

    bool has_prev = false;
    double yaw_unwrapped = 0.0;
    Pose prev_pose{};

    for (double t = 0.0; t <= max_t + 1e-9; t += sample_step)
    {
      Pose pose = timetable.get_pose(entity, t);
      if (!has_prev)
      {
        yaw_unwrapped = pose.yaw;
        has_prev = true;
      }
      else
      {
        yaw_unwrapped += pi_2_pi(pose.yaw - prev_pose.yaw);
      }

      ofs << csv_escape(scenario_label) << ","
          << csv_escape(name) << ","
          << t << ","
          << pose.x << ","
          << pose.y << ","
          << pose.yaw << ","
          << yaw_unwrapped << "\n";

      prev_pose = pose;
    }
  }

  std::cout << "[Log] Wrote timetable robot pose CSV: " << csv_path << std::endl;
}

void write_orientation_jump_csv(const std::string &csv_path,
                                const std::vector<OrientationJumpRecord> &records)
{
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open orientation jump CSV: "
              << csv_path << std::endl;
    return;
  }

  ofs << "scenario,source,robot,prev_time,curr_time,prev_x,prev_y,prev_yaw,curr_x,curr_y,curr_yaw,raw_yaw_delta,wrapped_yaw_delta,distance\n";
  ofs << std::fixed << std::setprecision(6);

  for (const auto &record : records)
  {
    ofs << csv_escape(record.scenario_label) << ","
        << csv_escape(record.source) << ","
        << csv_escape(record.robot_name) << ","
        << record.prev_time << ","
        << record.curr_time << ","
        << record.prev_pose.x << ","
        << record.prev_pose.y << ","
        << record.prev_pose.yaw << ","
        << record.curr_pose.x << ","
        << record.curr_pose.y << ","
        << record.curr_pose.yaw << ","
        << record.raw_yaw_delta << ","
        << record.wrapped_yaw_delta << ","
        << record.distance << "\n";
  }

  std::cout << "[Log] Wrote orientation jump CSV: " << csv_path << std::endl;
}

std::vector<OrientationJumpRecord> collect_robot_orientation_jumps(
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    double dense_sample_step,
    double wrapped_jump_threshold)
{
  std::vector<OrientationJumpRecord> out;
  const double max_t = timetable.get_max_time();

  auto maybe_append = [&](const std::string &source,
                          const std::string &robot_name,
                          double prev_time,
                          const Pose &prev_pose,
                          double curr_time,
                          const Pose &curr_pose)
  {
    const double raw_yaw_delta = curr_pose.yaw - prev_pose.yaw;
    const double wrapped_yaw_delta = pi_2_pi(raw_yaw_delta);
    if (std::abs(wrapped_yaw_delta) < wrapped_jump_threshold)
      return;

    OrientationJumpRecord record;
    record.scenario_label = scenario_label;
    record.source = source;
    record.robot_name = robot_name;
    record.prev_time = prev_time;
    record.curr_time = curr_time;
    record.prev_pose = prev_pose;
    record.curr_pose = curr_pose;
    record.raw_yaw_delta = raw_yaw_delta;
    record.wrapped_yaw_delta = wrapped_yaw_delta;
    record.distance = std::hypot(curr_pose.x - prev_pose.x,
                                 curr_pose.y - prev_pose.y);
    out.push_back(record);
  };

  for (const auto &[name, entity] : entities)
  {
    if (!entity || entity->type != EntityType::ROBOT)
      continue;

    bool has_prev_dense = false;
    double prev_dense_time = 0.0;
    Pose prev_dense_pose{};
    for (double t = 0.0; t <= max_t + 1e-9; t += dense_sample_step)
    {
      const Pose pose = timetable.get_pose(entity, t);
      if (has_prev_dense)
      {
        maybe_append("dense-get-pose", name,
                     prev_dense_time, prev_dense_pose,
                     t, pose);
      }
      prev_dense_time = t;
      prev_dense_pose = pose;
      has_prev_dense = true;
    }

    const auto &db = timetable.get_database();
    auto it = db.find(entity);
    if (it == db.end())
      continue;

    bool has_prev_stored = false;
    double prev_stored_time = 0.0;
    Pose prev_stored_pose{};
    for (const auto &[t, pose] : it->second)
    {
      if (has_prev_stored)
      {
        maybe_append("stored-samples", name,
                     prev_stored_time, prev_stored_pose,
                     t, pose);
      }
      prev_stored_time = t;
      prev_stored_pose = pose;
      has_prev_stored = true;
    }
  }

  std::sort(out.begin(), out.end(),
            [](const OrientationJumpRecord &a, const OrientationJumpRecord &b)
            {
              const double a_mag = std::abs(a.wrapped_yaw_delta);
              const double b_mag = std::abs(b.wrapped_yaw_delta);
              if (std::abs(a_mag - b_mag) > 1e-9)
                return a_mag > b_mag;
              if (std::abs(a.curr_time - b.curr_time) > 1e-9)
                return a.curr_time < b.curr_time;
              if (a.robot_name != b.robot_name)
                return a.robot_name < b.robot_name;
              return a.source < b.source;
            });
  return out;
}

void log_timetable_orientation_diagnostics(
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params)
{
  const std::string label_slug = sanitize_filename_component(scenario_label);
  const std::string pose_csv =
      std::string(CMAKE_SOURCE_DIR) + "/timetable_robot_pose_log_" +
      label_slug + ".csv";
  const std::string jump_csv =
      std::string(CMAKE_SOURCE_DIR) + "/timetable_robot_yaw_jumps_" +
      label_slug + ".csv";

  const double sample_step = std::max(1e-3, std::min(0.05, shared_collision_check_step(params)));
  const double jump_threshold = std::max(0.75, params.yaw_resolution * 2.0);

  write_timetable_robot_pose_csv(pose_csv, scenario_label, timetable, entities,
                                 sample_step);
  auto jump_records =
      collect_robot_orientation_jumps(scenario_label, timetable, entities,
                                      sample_step, jump_threshold);
  write_orientation_jump_csv(jump_csv, jump_records);

  if (jump_records.empty())
  {
    std::cout << "[Diag] No suspicious robot yaw jumps found in scenario "
              << scenario_label << " using threshold "
              << std::fixed << std::setprecision(2)
              << jump_threshold << " rad." << std::endl;
    return;
  }

  std::cout << "[Diag] Found " << jump_records.size()
            << " suspicious robot yaw jumps in scenario "
            << scenario_label << "." << std::endl;
  const std::size_t preview_count = std::min<std::size_t>(5, jump_records.size());
  for (std::size_t i = 0; i < preview_count; ++i)
  {
    const auto &record = jump_records[i];
    std::cout << "        [" << record.source << "] "
              << record.robot_name << " t=" << std::fixed << std::setprecision(2)
              << record.prev_time << " -> " << record.curr_time
              << " yaw=" << record.prev_pose.yaw << " -> " << record.curr_pose.yaw
              << " wrapped_delta=" << record.wrapped_yaw_delta
              << " raw_delta=" << record.raw_yaw_delta
              << " distance=" << record.distance << std::endl;
  }
}

std::string sanitize_filename_component(const std::string &label)
{
  std::string out;
  out.reserve(label.size());

  for (char c : label)
  {
    unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc))
      out.push_back(static_cast<char>(std::tolower(uc)));
    else
      out.push_back('_');
  }

  while (!out.empty() && out.front() == '_')
    out.erase(out.begin());
  while (!out.empty() && out.back() == '_')
    out.pop_back();

  if (out.empty())
    out = "scenario";
  return out;
}

std::string default_result_summary_path(const ReloPush::HandoffInstanceInfo &instance_info,
                                        const std::string &scenario_label)
{
  return std::string(CMAKE_SOURCE_DIR) + "/results/mars_result_summary_" +
         sanitize_filename_component(instance_info.file_name) + "_ind" +
         std::to_string(instance_info.instance_index) + "_" +
         sanitize_filename_component(scenario_label) + ".png";
}

std::vector<EntityMeta *> sorted_entities_by_type(
    const std::unordered_map<std::string, EntityMeta *> &entities,
    EntityType type)
{
  std::vector<EntityMeta *> out;
  for (const auto &[name, entity] : entities)
  {
    if (entity && entity->type == type)
      out.push_back(entity);
  }
  std::sort(out.begin(), out.end(),
            [](const EntityMeta *a, const EntityMeta *b)
            {
              return a->name < b->name;
            });
  return out;
}

QColor color_from_hex_or_default(const std::string &hex, const QColor &fallback)
{
  QColor color(QString::fromStdString(hex));
  return color.isValid() ? color : fallback;
}

void draw_entity_polygon(QPainter &painter,
                         const Pose &pose,
                         const EntityMeta *entity,
                         const QColor &fill,
                         const QPen &outline,
                         const std::function<QPointF(double, double)> &to_screen,
                         const QString &label = QString())
{
  if (!entity)
    return;

  QPolygonF polygon;
  const auto corners = get_corners(pose.x, pose.y, pose.yaw,
                                   entity->size.front_length,
                                   entity->size.rear_length,
                                   entity->size.width);
  for (const auto &corner : corners)
    polygon << to_screen(corner.x, corner.y);

  painter.setBrush(fill);
  painter.setPen(outline);
  painter.drawPolygon(polygon);

  if (entity->type == EntityType::ROBOT)
  {
    const QPointF center = to_screen(pose.x, pose.y);
    const QPointF front = to_screen(
        pose.x + entity->size.front_length * std::cos(pose.yaw),
        pose.y + entity->size.front_length * std::sin(pose.yaw));
    painter.setPen(QPen(outline.color(), 2));
    painter.drawLine(center, front);
  }

  if (!label.isEmpty())
  {
    if (QApplication::instance())
    {
      painter.setPen(QPen(QColor("#202020"), 1));
      painter.drawText(to_screen(pose.x, pose.y) + QPointF(4, -4), label);
    }
  }
}

void draw_result_summary_pane_base(QPainter &painter,
                                   const QRectF &pane,
                                   const Params &params,
                                   const QString &title,
                                   const std::function<QPointF(double, double)> &to_screen)
{
  painter.fillRect(pane, QColor("#FFFFFF"));

  painter.setPen(QPen(QColor("#E7E1D8"), 1));
  const double grid_step = std::max(0.1, params.xy_resolution * 5.0);
  for (double x = params.min_x; x <= params.max_x + 1e-9; x += grid_step)
  {
    painter.drawLine(to_screen(x, params.min_y), to_screen(x, params.max_y));
  }
  for (double y = params.min_y; y <= params.max_y + 1e-9; y += grid_step)
  {
    painter.drawLine(to_screen(params.min_x, y), to_screen(params.max_x, y));
  }

  painter.setPen(QPen(QColor("#1F2933"), 2));
  painter.setBrush(Qt::NoBrush);
  const QPointF top_left = to_screen(params.min_x, params.max_y);
  const QPointF bottom_right = to_screen(params.max_x, params.min_y);
  painter.drawRect(QRectF(top_left, bottom_right).normalized());

  if (QApplication::instance())
  {
    painter.setPen(QPen(QColor("#111827"), 1));
    QFont title_font = painter.font();
    title_font.setBold(true);
    title_font.setPointSize(13);
    painter.setFont(title_font);
    painter.drawText(pane.adjusted(4, 4, -4, -4), Qt::AlignLeft | Qt::AlignTop, title);
  }
}

void draw_start_configuration_pane(
    QPainter &painter,
    const QRectF &pane,
    const Params &params,
    const TimeTable &timetable,
    const std::vector<EntityMeta *> &robots,
    const std::vector<EntityMeta *> &objects,
    const std::vector<QColor> &robot_colors,
    const std::function<QPointF(double, double)> &to_screen)
{
  draw_result_summary_pane_base(painter, pane, params,
                                "Starting configuration", to_screen);

  for (EntityMeta *object : objects)
  {
    draw_entity_polygon(painter, timetable.get_pose(object, 0.0), object,
                        QColor(137, 176, 174, 170),
                        QPen(QColor("#587775"), 1),
                        to_screen,
                        QString::fromStdString(object->name));
  }

  for (std::size_t i = 0; i < robots.size(); ++i)
  {
    QColor color = robot_colors.empty()
                       ? QColor("#D55E00")
                       : robot_colors[i % robot_colors.size()];
    color.setAlpha(210);
    draw_entity_polygon(painter, timetable.get_pose(robots[i], 0.0), robots[i],
                        color,
                        QPen(QColor("#1F2933"), 1),
                        to_screen,
                        QString::fromStdString(robots[i]->name));
  }
}

void draw_result_traces_pane(
    QPainter &painter,
    const QRectF &pane,
    const Params &params,
    const TimeTable &timetable,
    const std::vector<EntityMeta *> &robots,
    const std::vector<EntityMeta *> &objects,
    const std::vector<QColor> &robot_colors,
    const std::function<QPointF(double, double)> &to_screen)
{
  draw_result_summary_pane_base(painter, pane, params,
                                "Robot traces and final object configuration",
                                to_screen);

  const double max_time = timetable.get_max_time();

  for (EntityMeta *object : objects)
  {
    draw_entity_polygon(painter, object->initial_pose, object,
                        QColor(137, 176, 174, 70),
                        QPen(QColor(88, 119, 117, 120), 1, Qt::DashLine),
                        to_screen);
  }

  const auto &database = timetable.get_database();
  for (std::size_t i = 0; i < robots.size(); ++i)
  {
    EntityMeta *robot = robots[i];
    QColor color = robot_colors.empty()
                       ? QColor("#D55E00")
                       : robot_colors[i % robot_colors.size()];
    auto it = database.find(robot);
    if (it == database.end() || it->second.empty())
      continue;

    QPainterPath trace;
    bool has_point = false;
    for (const auto &[time, pose] : it->second)
    {
      const QPointF point = to_screen(pose.x, pose.y);
      if (!has_point)
      {
        trace.moveTo(point);
        has_point = true;
      }
      else
      {
        trace.lineTo(point);
      }
    }

    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(color, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPath(trace);

    Pose final_pose = timetable.get_pose(robot, max_time);
    QColor body_color = color;
    body_color.setAlpha(210);
    draw_entity_polygon(painter, final_pose, robot,
                        body_color,
                        QPen(QColor("#1F2933"), 1),
                        to_screen,
                        QString::fromStdString(robot->name));
  }

  for (EntityMeta *object : objects)
  {
    Pose final_pose = timetable.get_pose(object, max_time);
    draw_entity_polygon(painter, final_pose, object,
                        QColor(66, 92, 89, 220),
                        QPen(QColor("#223A38"), 1),
                        to_screen,
                        QString::fromStdString(object->name));
  }
}

void export_result_summary_figure(
    const RuntimeOptions &options,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params)
{
  if (!options.enable_result_summary_figure)
    return;

  const std::string output_path =
      options.result_summary_output_path.empty()
          ? default_result_summary_path(instance_info, scenario_label)
          : options.result_summary_output_path;

  std::filesystem::path output_fs_path(output_path);
  if (output_fs_path.has_parent_path())
    std::filesystem::create_directories(output_fs_path.parent_path());

  constexpr int image_width = 1400;
  constexpr int pane_height = 640;
  constexpr int outer_margin = 48;
  const int subplot_gap = static_cast<int>(
      std::round(std::max(0.0, options.result_summary_subplot_gap)));
  const int image_height = outer_margin * 2 + pane_height * 2 + subplot_gap;

  QImage image(image_width, image_height, QImage::Format_ARGB32_Premultiplied);
  image.fill(QColor("#F8F5EF"));
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF top_pane(outer_margin, outer_margin,
                        image_width - outer_margin * 2, pane_height);
  const QRectF bottom_pane(outer_margin,
                           outer_margin + pane_height + subplot_gap,
                           image_width - outer_margin * 2,
                           pane_height);

  auto make_transform = [&](const QRectF &pane)
  {
    const double world_w = std::max(1e-9, params.max_x - params.min_x);
    const double world_h = std::max(1e-9, params.max_y - params.min_y);
    const double inset = 36.0;
    const double scale = std::min((pane.width() - inset * 2.0) / world_w,
                                  (pane.height() - inset * 2.0) / world_h);
    const double x_offset = pane.left() + (pane.width() - world_w * scale) * 0.5;
    const double y_offset = pane.top() + (pane.height() - world_h * scale) * 0.5;
    return [=](double x, double y)
    {
      return QPointF(x_offset + (x - params.min_x) * scale,
                     y_offset + (params.max_y - y) * scale);
    };
  };

  std::vector<EntityMeta *> robots = sorted_entities_by_type(entities, EntityType::ROBOT);
  std::vector<EntityMeta *> objects = sorted_entities_by_type(entities, EntityType::OBJECT);
  std::vector<QColor> robot_colors;
  robot_colors.reserve(options.robot_trace_colors.size());
  for (std::size_t i = 0; i < options.robot_trace_colors.size(); ++i)
  {
    static const std::vector<QColor> fallback = {
        QColor("#D55E00"), QColor("#0072B2"), QColor("#009E73"),
        QColor("#CC79A7"), QColor("#E69F00"), QColor("#56B4E9")};
    robot_colors.push_back(color_from_hex_or_default(
        options.robot_trace_colors[i], fallback[i % fallback.size()]));
  }

  draw_start_configuration_pane(painter, top_pane, params, timetable,
                                robots, objects, robot_colors,
                                make_transform(top_pane));
  draw_result_traces_pane(painter, bottom_pane, params, timetable,
                          robots, objects, robot_colors,
                          make_transform(bottom_pane));

  painter.end();
  if (!image.save(QString::fromStdString(output_path)))
  {
    std::cerr << "[ResultViz] Failed to write result summary figure: "
              << output_path << std::endl;
    return;
  }

  std::cout << "[ResultViz] Wrote result summary figure: "
            << output_path << std::endl;
}

bool poses_approximately_equal(const Pose &a, const Pose &b,
                               double pos_tol = 1e-3,
                               double yaw_tol = 1e-3)
{
  return std::hypot(a.x - b.x, a.y - b.y) <= pos_tol &&
         std::abs(pi_2_pi(a.yaw - b.yaw)) <= yaw_tol;
}

std::string format_pose_compact(const Pose &pose)
{
  std::ostringstream oss;
  oss << "("
      << std::fixed << std::setprecision(2)
      << pose.x << ", " << pose.y << ", " << pose.yaw << ")";
  return oss.str();
}

std::string describe_entity_pose_context(const TimeTable &timetable,
                                         EntityMeta *entity,
                                         double t)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);

  if (!entity)
  {
    oss << "<null entity>";
    return oss.str();
  }

  const Pose pose_at_t = timetable.get_pose(entity, t);
  oss << entity->name << " @ t=" << t
      << " pose=" << format_pose_compact(pose_at_t);

  const auto &db = timetable.get_database();
  auto ent_it = db.find(entity);
  if (ent_it == db.end() || ent_it->second.empty())
  {
    oss << " | no timetable samples";
    return oss.str();
  }

  const auto &samples = ent_it->second;
  auto next_it = samples.lower_bound(t);

  if (next_it != samples.begin())
  {
    auto prev_it = std::prev(next_it);
    oss << " | prev=(" << prev_it->first << ", "
        << format_pose_compact(prev_it->second) << ")";
  }
  else
  {
    oss << " | prev=(none)";
  }

  if (next_it != samples.end())
  {
    oss << " | next=(" << next_it->first << ", "
        << format_pose_compact(next_it->second) << ")";
  }
  else
  {
    auto last_it = std::prev(samples.end());
    oss << " | next=(end, " << last_it->first << ", "
        << format_pose_compact(last_it->second) << ")";
  }

  if (next_it != samples.begin() && next_it != samples.end())
  {
    const auto prev_it = std::prev(next_it);
    const double dt = next_it->first - prev_it->first;
    const double dist = std::hypot(next_it->second.x - prev_it->second.x,
                                   next_it->second.y - prev_it->second.y);
    if (dt > 1e-6)
    {
      oss << " | local_dt=" << dt
          << " | local_move=" << dist;
    }

    if (poses_approximately_equal(prev_it->second, next_it->second))
    {
      oss << " | stationary-window";
    }
  }

  return oss.str();
}

std::string summarize_pair_collision_window(const TimeTable &timetable,
                                            EntityMeta *entity_a,
                                            EntityMeta *entity_b,
                                            const Params &params,
                                            double center_time,
                                            double horizon = 2.0,
                                            double step = 0.05)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);

  if (!entity_a || !entity_b)
  {
    oss << "pair window unavailable";
    return oss.str();
  }

  const double start_t = std::max(0.0, center_time - horizon);
  const double end_t = center_time + horizon;

  bool in_collision = false;
  double collision_start = -1.0;
  double collision_end = -1.0;
  int collision_samples = 0;

  for (double t = start_t; t <= end_t + 1e-9; t += step)
  {
    const Pose pose_a = timetable.get_pose(entity_a, t);
    const Pose pose_b = timetable.get_pose(entity_b, t);
    const Corners corners_a =
        get_collision_corners_for_type(pose_a, entity_a->type, entity_a->size, params);
    const Corners corners_b =
        get_collision_corners_for_type(pose_b, entity_b->type, entity_b->size, params);
    const bool colliding = rectangles_intersect(corners_a, corners_b);

    if (colliding)
    {
      if (!in_collision)
      {
        in_collision = true;
        collision_start = t;
      }
      collision_end = t;
      collision_samples += 1;
    }
    else if (in_collision)
    {
      break;
    }
  }

  if (collision_samples == 0)
  {
    oss << "No repeated pair collision found in ["
        << start_t << ", " << end_t << "]";
    return oss.str();
  }

  oss << "Pair collision window ~[" << collision_start
      << ", " << collision_end << "]"
      << " sampled every " << step << "s"
      << " (" << collision_samples << " samples)";
  return oss.str();
}

std::string compare_pair_collision_checks(const TimeTable &timetable,
                                          EntityMeta *entity_a,
                                          EntityMeta *entity_b,
                                          const Params &params,
                                          double t)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);

  if (!entity_a || !entity_b)
  {
    oss << "pair check unavailable";
    return oss.str();
  }

  const Pose pose_a = timetable.get_pose(entity_a, t);
  const Pose pose_b = timetable.get_pose(entity_b, t);
  const CollisionGeometry geom_a =
      setup_collision_geometry_for_type(pose_a, entity_a->type, entity_a->size, params);
  const auto fast_check = check_entity_collision(geom_a, pose_a, entity_b, pose_b, params);
  const Corners corners_a =
      get_collision_corners_for_type(pose_a, entity_a->type, entity_a->size, params);
  const Corners corners_b =
      get_collision_corners_for_type(pose_b, entity_b->type, entity_b->size, params);
  const bool direct_overlap = rectangles_intersect(corners_a, corners_b);

  const double dist = std::hypot(pose_a.x - pose_b.x, pose_a.y - pose_b.y);
  const double other_inflation =
      collision_inflation_for_type(entity_b->type, params);
  const double other_diag =
      collision_origin_radius(entity_b->size, other_inflation);
  const double fast_threshold =
      geom_a.diagonal_radius + other_diag + params.safety_margin;

  oss << "pair direct_overlap=" << (direct_overlap ? "true" : "false")
      << ", fast_check=" << (fast_check.has_collision ? "true" : "false")
      << ", center_dist=" << dist
      << ", fast_threshold=" << fast_threshold;
  return oss.str();
}

void write_allocation_search_summary_csv(
    const std::string &csv_path,
    const std::vector<AllocationRunSummary> &summaries,
    double greedy_makespan)
{
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open allocation summary CSV: " << csv_path << std::endl;
    return;
  }

  ofs << "scenario,feasible,makespan,delta_vs_greedy,successful_tasks,failed_tasks,parking_seed\n";
  ofs << std::fixed << std::setprecision(2);

  for (const auto &summary : summaries)
  {
    double delta = (std::isfinite(summary.makespan) && std::isfinite(greedy_makespan))
                       ? (summary.makespan - greedy_makespan)
                       : std::numeric_limits<double>::infinity();
    ofs << csv_escape(summary.label) << ","
        << (summary.all_tasks_succeeded ? "TRUE" : "FALSE") << ",";
    if (std::isfinite(summary.makespan))
      ofs << summary.makespan;
    else
      ofs << "INF";
    ofs << ",";
    if (std::isfinite(delta))
      ofs << delta;
    else
      ofs << "INF";
    ofs << ","
        << summary.successful_tasks << ","
        << summary.failed_tasks << ","
        << summary.parking_seed << "\n";
  }

  std::cout << "[Log] Wrote allocation comparison CSV: " << csv_path << std::endl;
}

void write_search_trial_records_csv(
    const std::string &csv_path,
    const std::vector<SearchTrialRecord> &records)
{
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open search trial CSV: " << csv_path << std::endl;
    return;
  }

  ofs << "search_label,iteration,feasible,makespan,successful_tasks,failed_tasks,parking_seed,order\n";
  ofs << std::fixed << std::setprecision(2);

  for (const auto &record : records)
  {
    ofs << csv_escape(record.search_label) << ","
        << record.iteration << ","
        << (record.feasible ? "TRUE" : "FALSE") << ",";
    if (std::isfinite(record.makespan))
      ofs << record.makespan;
    else
      ofs << "INF";
    ofs << ","
        << record.successful_tasks << ","
        << record.failed_tasks << ","
        << record.parking_seed << ","
        << csv_escape(record.order_description) << "\n";
  }

  std::cout << "[Log] Wrote search trial CSV: " << csv_path << std::endl;
}

void write_instance_run_record_csv(
    const std::string &csv_path,
    const ReloPush::HandoffInstanceInfo &instance_info,
    double relopush_single_robot_makespan,
    double greedy_makespan,
    double lns_best_makespan,
    int lns_iterations,
    const std::string &best_overall_label,
    double best_overall_makespan)
{
  bool write_header = false;
  {
    std::ifstream ifs(csv_path);
    write_header = !ifs.good() || ifs.peek() == std::ifstream::traits_type::eof();
  }

  std::ofstream ofs(csv_path, std::ios::app);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open instance record CSV: " << csv_path << std::endl;
    return;
  }

  if (write_header)
  {
    ofs << "file_name,instance_index,relopush_single_robot_makespan,"
           "greedy_allocation_makespan,lns_best_makespan,lns_iterations,"
           "best_overall_label,best_overall_makespan\n";
  }

  ofs << std::fixed << std::setprecision(2);
  ofs << csv_escape(instance_info.file_name) << ","
      << instance_info.instance_index << ",";
  if (std::isfinite(relopush_single_robot_makespan))
    ofs << relopush_single_robot_makespan;
  else
    ofs << "INF";
  ofs << ",";
  if (std::isfinite(greedy_makespan))
    ofs << greedy_makespan;
  else
    ofs << "INF";
  ofs << ",";
  if (std::isfinite(lns_best_makespan))
    ofs << lns_best_makespan;
  else
    ofs << "INF";
  ofs << ","
      << lns_iterations << ","
      << csv_escape(best_overall_label) << ",";
  if (std::isfinite(best_overall_makespan))
    ofs << best_overall_makespan;
  else
    ofs << "INF";
  ofs << "\n";

  std::cout << "[Log] Appended instance record CSV: " << csv_path << std::endl;
}

double compute_relopush_single_robot_makespan(
    const std::vector<FinalAllocation> &loaded_sequence)
{
  constexpr float v_p = 0.28f;
  constexpr float v_np = 0.35f;
  constexpr float v_backward = -0.3f;

  auto compute_segment_duration =
      [&](const ReloPush::StatePathPtr &path_ptr, bool is_pushing) -> double
  {
    if (!path_ptr || path_ptr->empty())
    {
      return 0.0;
    }

    ReloPush::State previous_state = path_ptr->front();
    double elapsed = 0.0;
    bool is_prev_forward = true;

    for (std::size_t i = 1; i < path_ptr->size(); ++i)
    {
      const ReloPush::State &next_state = path_ptr->at(i);
      const double dx = next_state.x - previous_state.x;
      const double dy = next_state.y - previous_state.y;
      const double dist = std::sqrt(dx * dx + dy * dy);

      const double heading_x = std::cos(previous_state.yaw);
      const double heading_y = std::sin(previous_state.yaw);
      const double dot = dx * heading_x + dy * heading_y;

      bool add_wait = false;
      double chosen_vel = 0.0;
      if (is_pushing)
      {
        chosen_vel = v_p;
      }
      else if (dot >= 0.0)
      {
        chosen_vel = v_np;
        if (!is_prev_forward)
          add_wait = true;
        is_prev_forward = true;
      }
      else
      {
        chosen_vel = v_backward;
        if (is_prev_forward)
          add_wait = true;
        is_prev_forward = false;
      }

      const double speed = std::fabs(chosen_vel);
      const double dt = (speed > 1e-6) ? (dist / speed) : 0.0;
      elapsed += dt;
      if (add_wait)
      {
        elapsed += 0.8;
      }

      previous_state = next_state;
    }

    return elapsed;
  };

  double total_duration = 0.0;
  for (const auto &allocation : loaded_sequence)
  {
    total_duration += compute_segment_duration(allocation.firstApproachPath, false);

    if (allocation.obsReloPaths)
    {
      for (const auto &obs_path : *allocation.obsReloPaths)
      {
        auto path_ptr = obs_path.toStatePath();
        if (path_ptr && !path_ptr->empty())
        {
          path_ptr->at(path_ptr->size() - 1) =
              path_ptr->back().get_postPush(Constants::obs_relo_offset);
        }
        total_duration += compute_segment_duration(path_ptr, obs_path.is_pushing);
      }
    }

    for (std::size_t i = 0; i < allocation.paths.size(); ++i)
    {
      const auto &edge_group = allocation.paths[i];
      for (std::size_t path_idx = 0; path_idx < edge_group.paths.size(); ++path_idx)
      {
        auto edge_path = edge_group.paths[path_idx]->toStatePath();
        if (edge_path && !edge_path->empty() &&
            edge_group.paths.size() > 1 && path_idx == 0)
        {
          edge_path->push_back(
              edge_path->back().get_postPush(Constants::additional_push_dist));
        }
        total_duration += compute_segment_duration(
            edge_path,
            edge_group.paths[path_idx]->is_pushing);
      }

      if (allocation.edgeTransitPaths.size() > i &&
          !allocation.edgeTransitPaths.empty())
      {
        total_duration += compute_segment_duration(
            allocation.edgeTransitPaths[i],
            false);
      }
    }
  }

  return total_duration;
}

RobotMeta *make_relopush_robot()
{
  RobotMeta *robot = new RobotMeta;
  robot->name = "ReloPush-BOSS single robot";
  robot->type = EntityType::ROBOT;
  robot->size.front_length = 0.36;
  robot->size.rear_length = 0.12;
  robot->size.width = 0.275;
  robot->min_turning_radius = 1.43;
  robot->min_turning_radius_transit = 1.02;
  robot->min_turning_radius_transfer = 1.43;
  robot->wheel_base = 0.29;
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;
  return robot;
}

bool find_first_relopush_robot_pose(const std::vector<FinalAllocation> &loaded_sequence,
                                    Pose &out_pose)
{
  for (const auto &allocation : loaded_sequence)
  {
    if (allocation.firstApproachPath && !allocation.firstApproachPath->empty())
    {
      out_pose = PoseFromReloPushState(allocation.firstApproachPath->front());
      return true;
    }

    if (allocation.obsReloPaths)
    {
      for (const auto &obs_path : *allocation.obsReloPaths)
      {
        auto path = obs_path.toStatePath();
        if (path && !path->empty())
        {
          out_pose = PoseFromReloPushState(path->front());
          return true;
        }
      }
    }

    for (const auto &edge_group : allocation.paths)
    {
      for (const auto &edge_path : edge_group.paths)
      {
        if (!edge_path)
          continue;
        auto path = edge_path->toStatePath();
        if (path && !path->empty())
        {
          out_pose = PoseFromReloPushState(path->front());
          return true;
        }
      }
    }

    for (const auto &path : allocation.edgeTransitPaths)
    {
      if (path && !path->empty())
      {
        out_pose = PoseFromReloPushState(path->front());
        return true;
      }
    }
  }

  return false;
}

Trajectory make_relopush_trajectory(const ReloPush::StatePathPtr &path,
                                    bool is_transfer,
                                    RobotMeta *robot,
                                    EntityMeta *transferred_object,
                                    double start_time)
{
  Trajectory traj;
  traj.entity = robot;
  traj.transferred_object = transferred_object;
  traj.start_time = start_time;
  traj.is_transfer = is_transfer;
  traj.kind = is_transfer ? TrajectoryKind::TRANSFER : TrajectoryKind::TRANSIT;

  if (!path || path->empty())
    return traj;

  traj.waypoints.reserve(path->size());
  for (const auto &state : *path)
  {
    Waypoint wp = WaypointFromReloPushState(state);
    wp.time = 0.0;
    wp.linear_velocity = is_transfer ? robot->speed_transfer
                                     : robot->speed_transit;
    traj.waypoints.push_back(wp);
  }

  traj.CalcualteTimeStamps(robot);
  return traj;
}

void append_relopush_trajectory(TimeTable &timetable,
                                RobotMeta *robot,
                                EntityMeta *transferred_object,
                                const ReloPush::StatePathPtr &path,
                                bool is_transfer,
                                double &current_time)
{
  Trajectory traj = make_relopush_trajectory(
      path, is_transfer, robot, transferred_object, current_time);
  if (traj.waypoints.empty())
    return;

  timetable.add_trajectory(traj);
  current_time += traj.waypoints.back().time;
}

void append_relopush_edge_path(TimeTable &timetable,
                               RobotMeta *robot,
                               EntityMeta *transferred_object,
                               const EdgePath &edge_path,
                               double &current_time)
{
  append_relopush_trajectory(timetable, robot, transferred_object,
                             edge_path.toStatePath(),
                             edge_path.is_pushing,
                             current_time);
}

void visualize_relopush_plan(
    int argc,
    char **argv,
    const std::vector<FinalAllocation> &loaded_sequence,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const RuntimeOptions &options)
{
  if (loaded_sequence.empty())
    return;

  std::vector<std::unique_ptr<EntityMeta>> entity_storage;
  std::unordered_map<std::string, EntityMeta *> entities;

  RobotMeta *robot = make_relopush_robot();
  Pose first_robot_pose{};
  if (find_first_relopush_robot_pose(loaded_sequence, first_robot_pose))
    robot->initial_pose = first_robot_pose;
  entity_storage.emplace_back(robot);
  entities[robot->name] = robot;

  for (const auto &[name, info] : loaded_sequence.front().snapshot.mo_list)
  {
    ObjectMeta *obj = new ObjectMeta;
    obj->name = name;
    obj->type = EntityType::OBJECT;
    obj->initial_pose = {info.x, info.y, NormalizeReloPushYaw(info.nominalOrientation)};
    obj->size.front_length = 0.075;
    obj->size.rear_length = 0.075;
    obj->size.width = 0.15;
    entity_storage.emplace_back(obj);
    entities[name] = obj;
  }

  TimeTable timetable;
  timetable.add_initial(entities);

  double current_time = 0.0;
  for (const auto &allocation : loaded_sequence)
  {
    append_relopush_trajectory(timetable, robot, nullptr,
                               allocation.firstApproachPath,
                               false, current_time);

    if (allocation.obsReloPaths)
    {
      std::size_t obs_path_base_idx = 0;
      for (std::size_t obs_ind = 1;
           obs_ind + 1 < allocation.vertexChain.size();
           ++obs_ind)
      {
        const auto obs_it = entities.find(allocation.vertexChain[obs_ind].name);
        EntityMeta *obs_entity =
            (obs_it != entities.end()) ? obs_it->second : nullptr;

        const std::size_t push_path_idx = obs_path_base_idx;
        const std::size_t post_path_idx = obs_path_base_idx + 1;
        if (allocation.obsReloPaths->size() > push_path_idx)
        {
          append_relopush_edge_path(
              timetable, robot, obs_entity,
              allocation.obsReloPaths->at(push_path_idx), current_time);
        }
        if (allocation.obsReloPaths->size() > post_path_idx)
        {
          append_relopush_edge_path(
              timetable, robot, nullptr,
              allocation.obsReloPaths->at(post_path_idx), current_time);
        }
        obs_path_base_idx += 2;
      }
    }

    for (std::size_t edge_idx = 0; edge_idx < allocation.paths.size(); ++edge_idx)
    {
      const auto &edge_group = allocation.paths[edge_idx];
      EntityMeta *edge_object = nullptr;
      auto edge_object_it = entities.find(allocation.object.name);
      if (edge_object_it != entities.end())
      {
        edge_object = edge_object_it->second;
      }

      for (std::size_t path_idx = 0; path_idx < edge_group.paths.size(); ++path_idx)
      {
        if (!edge_group.paths[path_idx])
          continue;

        auto edge_path = edge_group.paths[path_idx]->toStatePath();
        if (edge_path && !edge_path->empty() &&
            edge_group.paths.size() > 1 && path_idx == 0)
        {
          edge_path->push_back(
              edge_path->back().get_postPush(Constants::additional_push_dist));
        }

        append_relopush_trajectory(
            timetable, robot, edge_object, edge_path,
            edge_group.paths[path_idx]->is_pushing, current_time);
      }

      if (allocation.edgeTransitPaths.size() > edge_idx &&
          allocation.edgeTransitPaths[edge_idx])
      {
        append_relopush_trajectory(
            timetable, robot, nullptr,
            allocation.edgeTransitPaths[edge_idx],
            false, current_time);
      }
    }
  }

  Params params = initialize_params(loaded_sequence, options);
  std::ostringstream context;
  context << "ReloPush-BOSS single-robot plan replay"
          << "\nInstance: " << instance_info.file_name
          << "\nIndex: " << instance_info.instance_index
          << "\nRobot dimensions: front=0.36, rear=0.12, width=0.275"
          << "\nSpeeds: transit=0.20, transfer=0.15";

  std::cout << "[Debug] Visualizing ReloPush-BOSS single-robot plan "
            << "before multi-robot allocation. Replay makespan="
            << std::fixed << std::setprecision(2)
            << timetable.get_max_time() << "s" << std::endl;
  show_results(argc, argv, timetable, entities, params);
}

ReloPush::HandoffInstanceInfo default_instance_info(const RuntimeOptions &options)
{
  ReloPush::HandoffInstanceInfo info;
  std::string input_path = options.input_sequence_path.empty()
                               ? default_sequence_path()
                               : options.input_sequence_path;
  const std::size_t slash_pos = input_path.find_last_of("/\\");
  info.file_name = (slash_pos == std::string::npos)
                       ? input_path
                       : input_path.substr(slash_pos + 1);
  info.instance_index = -1;
  return info;
}

std::uint32_t mix_seed(std::uint32_t seed, std::uint32_t salt)
{
  std::uint32_t x = seed ^ (salt + 0x9e3779b9u + (seed << 6) + (seed >> 2));
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

bool is_valid_transfer_contact(EntityMeta *e1, const Pose &p1,
                               EntityMeta *e2, const Pose &p2)
{
  RobotMeta *robot = dynamic_cast<RobotMeta *>(e1->type == EntityType::ROBOT ? e1 : e2);
  ObjectMeta *obj = dynamic_cast<ObjectMeta *>(e1->type == EntityType::OBJECT ? e1 : e2);
  if (!robot || !obj)
    return false;

  const Pose &r_pose = (e1 == robot) ? p1 : p2;
  const Pose &o_pose = (e1 == obj) ? p1 : p2;
  double dx = o_pose.x - r_pose.x;
  double dy = o_pose.y - r_pose.y;

  double fx = std::cos(r_pose.yaw);
  double fy = std::sin(r_pose.yaw);
  double forward = dx * fx + dy * fy;
  double lateral = -dx * fy + dy * fx;

  double expected = robot->size.front_length + obj->size.rear_length;
  if (std::abs(forward - expected) > 0.2)
    return false;
  if (std::abs(lateral) > 0.2)
    return false;

  return true;
}

std::string find_valid_start_contact_entity(
    RobotMeta *robot,
    const Pose &start_pose,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities);

bool has_active_transfer_contact(EntityMeta *e1, EntityMeta *e2, double t,
                                 const std::vector<TransferContactWindow> &windows)
{
  for (const auto &w : windows)
  {
    if (!w.robot || !w.object)
      continue;
    bool same_pair = (e1 == w.robot && e2 == w.object) ||
                     (e2 == w.robot && e1 == w.object);
    if (!same_pair)
      continue;
    if (w.start_time - 1e-6 <= t && t <= w.end_time + 1e-6)
      return true;
  }
  return false;
}

bool has_transfer_pair(EntityMeta *e1, EntityMeta *e2,
                       const std::vector<TransferContactWindow> &windows)
{
  for (const auto &w : windows)
  {
    if (!w.robot || !w.object)
      continue;
    bool same_pair = (e1 == w.robot && e2 == w.object) ||
                     (e2 == w.robot && e1 == w.object);
    if (same_pair)
      return true;
  }
  return false;
}

RobotMeta *find_robot_transferring_object_at_time(
    ObjectMeta *object,
    double t,
    TimeTable &timetable)
{
  if (!object)
    return nullptr;

  auto poses = timetable.get_poses(t);
  auto object_it = poses.find(object);
  if (object_it == poses.end())
    return nullptr;

  for (const auto &[ent, ent_pose] : poses)
  {
    if (!ent || ent->type != EntityType::ROBOT)
      continue;

    if (is_valid_transfer_contact(ent, ent_pose, object, object_it->second))
    {
      return dynamic_cast<RobotMeta *>(ent);
    }
  }

  return nullptr;
}

RobotMeta *find_resolvable_robot_blocker(
    const CollisionInfo &col_info,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto ent_it = entities.find(col_info.entity_name);
  if (ent_it == entities.end())
    return nullptr;

  EntityMeta *collider = ent_it->second;
  if (!collider)
    return nullptr;

  if (collider->type == EntityType::ROBOT)
  {
    return dynamic_cast<RobotMeta *>(collider);
  }

  if (collider->type == EntityType::OBJECT)
  {
    return find_robot_transferring_object_at_time(
        dynamic_cast<ObjectMeta *>(collider), col_info.time, timetable);
  }

  return nullptr;
}

double shared_collision_check_step(const Params &params);

TimeTableVerificationResult verify_timetable_collision_free(
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> &transfer_windows,
    double from_time = 0.0,
    double step = -1.0)
{
  TimeTableVerificationResult result;
  double max_t = timetable.get_max_time();
  if (step <= 0.0)
  {
    step = shared_collision_check_step(params);
  }

  for (double t = from_time; t <= max_t + 1e-9; t += step)
  {
    auto poses = timetable.get_poses(t);

    for (const auto &[ent, pose] : poses)
    {
      CollisionGeometry geom = setup_collision_geometry(pose, ent->size, 1.0);
      bool out_of_bounds = check_entity_bounds_collision(ent->type, pose, geom.corners, params);

      if (out_of_bounds)
      {
        result.is_valid = false;
        result.time = t;
        result.reason = "Boundary collision";
        result.entity_a = ent->name;
        result.entity_b = "Boundary";
        return result;
      }
    }

    std::vector<std::pair<EntityMeta *, Pose>> entries(poses.begin(), poses.end());
    for (size_t i = 0; i < entries.size(); ++i)
    {
      auto [e1, p1] = entries[i];
      CollisionGeometry g1 =
          setup_collision_geometry_for_type(p1, e1->type, e1->size, params);

      for (size_t j = i + 1; j < entries.size(); ++j)
      {
        auto [e2, p2] = entries[j];
        auto collision = check_entity_collision(g1, p1, e2, p2, params);
        if (!collision.has_collision)
          continue;

        if (has_active_transfer_contact(e1, e2, t, transfer_windows))
          continue;

        if (is_valid_transfer_contact(e1, p1, e2, p2))
          continue;

        bool robot_object_pair =
            (e1->type == EntityType::ROBOT && e2->type == EntityType::OBJECT) ||
            (e2->type == EntityType::ROBOT && e1->type == EntityType::OBJECT);
        if (robot_object_pair && has_transfer_pair(e1, e2, transfer_windows))
          continue;

        result.is_valid = false;
        result.time = t;
        result.reason = "Entity collision";
        result.entity_a = e1->name;
        result.entity_b = e2->name;
        return result;
      }
    }
  }

  return result;
}

// ==========================================
// 1. HELPER & UTILITY FUNCTIONS
// ==========================================

namespace
{
  double positive_or(double value, double fallback)
  {
    return value > 0.0 ? value : fallback;
  }

  int positive_or(int value, int fallback)
  {
    return value > 0 ? value : fallback;
  }
}

// --- Configuration & Initialization ---
Params initialize_params(const std::vector<FinalAllocation> &loadedSequence,
                         const RuntimeOptions &options)
{
  Params params;

  params.xy_resolution = positive_or(options.default_xy_resolution, params.xy_resolution);
  params.yaw_resolution = positive_or(options.default_yaw_resolution, params.yaw_resolution);
  params.time_step = positive_or(options.default_time_step, params.time_step);
  params.time_resolution = params.time_step;
  params.rs_step_size = positive_or(options.default_rs_step_size, params.rs_step_size);
  params.collision_steps = positive_or(options.default_collision_steps, params.collision_steps);
  params.collision_check_time_step =
      positive_or(options.default_collision_check_time_step, params.collision_check_time_step);
  params.max_steer = positive_or(options.default_max_steer, params.max_steer);
  params.turn_penalty = positive_or(options.default_turn_penalty, params.turn_penalty);
  params.reverse_penalty = positive_or(options.default_reverse_penalty, params.reverse_penalty);
  params.switch_penalty = positive_or(options.default_switch_penalty, params.switch_penalty);
  params.wait_penalty = positive_or(options.default_wait_penalty, params.wait_penalty);
  params.max_time = positive_or(options.default_max_time, params.max_time);
  params.final_push_distance =
      positive_or(options.default_final_push_distance, params.final_push_distance);
  params.inflation = positive_or(options.default_inflation, params.inflation);
  params.safety_margin = std::max(0.0, options.default_safety_margin);
  params.robot_collision_inflation =
      positive_or(options.default_robot_collision_inflation, params.robot_collision_inflation);

  // Keep the MARS analytic expansion threshold expressed relative to max steer.
  params.analytic_threshold =
      positive_or(options.default_analytic_threshold_scale, 5.0) * params.max_steer;
  // params.analytic_threshold = std::hypot(params.max_x - params.min_x,
  // params.max_y - params.min_y) * 2.0;

  // Set workspace boundaries if available
  if (!loadedSequence.empty())
  {
    const auto &bound = loadedSequence[0].snapshot.parameters.boundary;
    params.min_x = bound.xMin;
    params.min_y = bound.yMin;
    params.max_x = bound.xMax;
    params.max_y = bound.yMax;
    std::cout << "[Init] Workspace set: [" << bound.xMin << ", " << bound.xMax
              << "] x [" << bound.yMin << ", " << bound.yMax << "]"
              << std::endl;
  }
  return params;
}

std::unordered_map<std::string, EntityMeta *>
initialize_entities(const std::vector<FinalAllocation> &loadedSequence)
{
  std::unordered_map<std::string, EntityMeta *> entities;

  double common_front_length = 0.36;
  double common_rear_length = 0.12;
  double common_width = 0.275;
  double common_min_turning_radius_transit = 1.02;
  double common_min_turning_radius_transfer = 1.43;
  double common_wheel_base = 0.29;
  double common_speed_transit = 0.2;
  double common_speed_transfer = 0.15;

  // Robot 1
  RobotMeta *robot1 = new RobotMeta;
  robot1->name = "robot1";
  robot1->type = EntityType::ROBOT;
  robot1->initial_pose = {0.5, 0.45, 0.0};
  robot1->size.front_length = common_front_length;
  robot1->size.rear_length = common_rear_length;
  robot1->size.width = common_width;
  robot1->min_turning_radius = common_min_turning_radius_transfer;
  robot1->min_turning_radius_transit = common_min_turning_radius_transit;
  robot1->min_turning_radius_transfer = common_min_turning_radius_transfer;
  robot1->wheel_base = common_wheel_base;
  robot1->speed_transit = common_speed_transit;
  robot1->speed_transfer = common_speed_transfer;
  entities["robot1"] = robot1;

  // Robot 2
  RobotMeta *robot2 = new RobotMeta;
  robot2->name = "robot2";
  robot2->type = EntityType::ROBOT;
  robot2->initial_pose = {0.5, 3.0, 0.0};
  robot2->size.front_length = common_front_length;
  robot2->size.rear_length = common_rear_length;
  robot2->size.width = common_width;
  robot2->min_turning_radius = common_min_turning_radius_transfer;
  robot2->min_turning_radius_transit = common_min_turning_radius_transit;
  robot2->min_turning_radius_transfer = common_min_turning_radius_transfer;
  robot2->wheel_base = common_wheel_base;
  robot2->speed_transit = common_speed_transit;
  robot2->speed_transfer = common_speed_transfer;
  entities["robot2"] = robot2;

  // Robot 3
  RobotMeta *robot3 = new RobotMeta;
  robot3->name = "robot3";
  robot3->type = EntityType::ROBOT;
  robot3->initial_pose = {0.5, 4.5, 0.0};
  robot3->size.front_length = common_front_length;
  robot3->size.rear_length = common_rear_length;
  robot3->size.width = common_width;
  robot3->min_turning_radius = common_min_turning_radius_transfer;
  robot3->min_turning_radius_transit = common_min_turning_radius_transit;
  robot3->min_turning_radius_transfer = common_min_turning_radius_transfer;
  robot3->wheel_base = common_wheel_base;
  robot3->speed_transit = common_speed_transit;
  robot3->speed_transfer = common_speed_transfer;
  entities["robot3"] = robot3;

  // Parse Objects
  if (!loadedSequence.empty())
  {
    for (const auto &[name, info] : loadedSequence[0].snapshot.mo_list)
    {
      ObjectMeta *obj = new ObjectMeta;
      obj->name = name;
      obj->type = EntityType::OBJECT;
      obj->initial_pose = {info.x, info.y, info.nominalOrientation};
      // Assuming ObjectMeta size struct is similar
      obj->size.front_length = 0.075;
      obj->size.rear_length = 0.075;
      obj->size.width = 0.15;
      entities[name] = obj;
    }
  }
  return entities;
}

Pose calcRobotPoseFromObj(const Pose &obj_pose, const OccuRect &robot_size,
                          const OccuRect &obj_size)
{
  double offset = robot_size.front_length + obj_size.rear_length + 0.1 + 0.05;
  Pose robot_pose;
  robot_pose.x = obj_pose.x - offset * std::cos(obj_pose.yaw);
  robot_pose.y = obj_pose.y - offset * std::sin(obj_pose.yaw);
  robot_pose.yaw = obj_pose.yaw;
  return robot_pose;
}

double contact_offset_for_object(const RobotMeta *robot,
                                 const EntityMeta *object,
                                 double extra_clearance = 0.0)
{
  if (!robot || !object)
    return extra_clearance;

  return robot->size.front_length + object->size.rear_length + extra_clearance;
}

Pose recover_object_centric_pose_from_raw_terminal(const Pose &raw_terminal_pose,
                                                   double source_pre_push_distance)
{
  ReloPush::State raw_state(raw_terminal_pose.x, raw_terminal_pose.y,
                            mod2pi(raw_terminal_pose.yaw));
  return PoseFromReloPushState(raw_state.get_postPush(source_pre_push_distance));
}

Pose compute_adjusted_prepush_goal(const Pose &object_pose,
                                   double pushing_yaw,
                                   const RobotMeta *robot,
                                   const EntityMeta *object,
                                   double extra_clearance)
{
  if (!robot || !object)
  {
    Pose fallback = object_pose;
    fallback.yaw = mod2pi(pushing_yaw);
    return fallback;
  }

  ReloPush::State object_centric_pose(object_pose.x, object_pose.y,
                                      mod2pi(pushing_yaw));
  const double pre_push_distance =
      contact_offset_for_object(robot, object, extra_clearance);
  return PoseFromReloPushState(
      object_centric_pose.get_prePush(pre_push_distance));
}

Pose compute_adjusted_task_start_pose(const Task &task,
                                      const RobotMeta *robot,
                                      double query_time,
                                      const TimeTable &timetable)
{
  Pose fallback = task.TaskStartPoseRobot;
  if (!robot || !task.initialApproachEntity)
    return fallback;

  const Pose object_pose =
      timetable.get_pose(task.initialApproachEntity, query_time);
  return compute_adjusted_prepush_goal(
      object_pose, fallback.yaw, robot, task.initialApproachEntity, 0.01);
}

bool rewrite_transfer_terminal_pose(Trajectory *traj, RobotMeta *robot)
{
  if (!traj || !traj->is_transfer || !robot || !traj->transferred_object ||
      traj->waypoints.empty())
  {
    return false;
  }

  const double source_pre_push_distance =
      (traj->source_pre_push_distance > 1e-9)
          ? traj->source_pre_push_distance
          : contact_offset_for_object(robot, traj->transferred_object, 0.01);

  const Pose raw_terminal_pose = traj->waypoints.back();
  const Pose object_goal_pose = recover_object_centric_pose_from_raw_terminal(
      raw_terminal_pose, source_pre_push_distance);

  // Keep transfer endpoints at exact contact distance. Adding extra gap here
  // can be a problem if the robot is too short because retraction happens
  // after the push segment is committed.
  const Pose adjusted_terminal_pose = compute_adjusted_prepush_goal(
      object_goal_pose, raw_terminal_pose.yaw, robot, traj->transferred_object,
      0.0);

  Waypoint &last_waypoint = traj->waypoints.back();
  last_waypoint.x = adjusted_terminal_pose.x;
  last_waypoint.y = adjusted_terminal_pose.y;
  last_waypoint.yaw = adjusted_terminal_pose.yaw;
  traj->CalcualteTimeStamps(robot);
  return true;
}

// --- Collision & Blocking Checkers (Preserved from original) ---

// (Keep your helper functions project_corners, overlaps, get_rect_axes here)
// ... [Assuming SAT helpers are defined as in original] ...

// ==========================================
// SMARTER DIAGNOSTIC FUNCTION
// ==========================================
// ==========================================
void diagnose_planning_failure(RobotMeta *robot, const Pose &start,
                               const Pose &goal, double time,
                               TimeTable &timetable,
                               const std::unordered_map<std::string, EntityMeta *> &entities,
                               const Params &params)
{
  std::cerr << "\n  [Diagnostics] Analyzing failure for " << robot->name
            << " at t=" << time << "s..." << std::endl;

  auto others_map = timetable.get_poses(time);
  std::vector<std::pair<EntityMeta *, Pose>> others(others_map.begin(),
                                                    others_map.end());

  // --- Helper Lambda: Is this a valid transfer? ---
  // ... (existing lambda) ...

  std::cout << "    Start: (" << start.x << ", " << start.y << ", " << start.yaw << ")" << std::endl;
  std::cout << "    Goal:  (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")" << std::endl;

  // Cross-check with Planner's internal check
  PHAStar diag_planner(robot, goal, &timetable, &entities, params, false, "",
                       time, "Diagnostics");
  // Note: start node is initialized in constructor from robot->initial_pose
  if (diag_planner.start)
  {
    auto col_info = diag_planner.check_collision_at(diag_planner.start.get());
    std::cout << "    [Planner Check] Start Node Collision: " << (col_info.is_valid ? "VALID" : "COLLISION") << std::endl;
    if (!col_info.is_valid)
    {
      std::cout << "      Reason: " << col_info.reason
                << ", Entity: " << col_info.entity_name << std::endl;
    }

    // Also check Goal
    Node goal_node(goal.x, goal.y, goal.yaw, time + 10.0, 0, 0, nullptr, 0); // Arbitrary future time
    auto goal_info = diag_planner.check_collision_at(&goal_node);
    std::cout << "    [Planner Check] Goal Node Collision: " << (goal_info.is_valid ? "VALID" : "COLLISION") << std::endl;
    if (!goal_info.is_valid)
    {
      std::cout << "      Reason: " << goal_info.reason
                << ", Entity: " << goal_info.entity_name << std::endl;
    }
  }
  else
  {
    std::cout << "    [Planner Check] Start Node is NULL (Init failed?)" << std::endl;
  }

  auto is_valid_transfer = [](EntityMeta *e1, const Pose &p1, EntityMeta *e2,
                              const Pose &p2) -> bool
  {
    // 1. Identify Robot and Object
    RobotMeta *r =
        dynamic_cast<RobotMeta *>(e1->type == EntityType::ROBOT ? e1 : e2);
    ObjectMeta *o =
        dynamic_cast<ObjectMeta *>(e1->type == EntityType::OBJECT ? e1 : e2);
    if (!r || !o)
      return false; // Not a Robot-Object pair

    const Pose &r_pose = (e1 == r) ? p1 : p2;
    const Pose &o_pose = (e1 == o) ? p1 : p2;

    // 2. Check Orientation Alignment (should be similar for pushing)
    double yaw_diff = std::abs(r_pose.yaw - o_pose.yaw);
    while (yaw_diff > M_PI)
      yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI)
      yaw_diff += 2 * M_PI;
    if (std::abs(yaw_diff) > 0.5)
      return false; // Angle mismatch > ~30 deg

    // 3. Check Relative Position (Object should be in front)
    // Simple check: Distance should be roughly sum of half-lengths
    double dx = o_pose.x - r_pose.x;
    double dy = o_pose.y - r_pose.y;
    double dist = std::hypot(dx, dy);

    // Expected distance center-to-center approx (Front_R + Rear_O)
    // Allowing some tolerance (e.g., 0.2m)
    double expected_dist = r->size.front_length + o->size.rear_length;
    if (dist > expected_dist + 0.3 || dist < expected_dist - 0.3)
      return false;

    return true;
  };
  // ------------------------------------------------

  // 1. Check Start Pose Validity
  Corners start_c =
      get_corners(start.x, start.y, start.yaw, robot->size.front_length,
                  robot->size.rear_length, robot->size.width);
  bool start_ok = true;
  for (const auto &[ent, pose] : others)
  {
    if (ent == robot)
      continue;

    Corners ent_c =
        get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length,
                    ent->size.rear_length, ent->size.width);
    if (rectangles_intersect(start_c, ent_c))
    {
      std::cerr << "    [FAIL] Start Pose COLLIDES with " << ent->name
                << " (Dist: " << std::hypot(start.x - pose.x, start.y - pose.y)
                << "m)" << std::endl;
      start_ok = false;
    }
  }
  if (start_ok)
    std::cerr << "    [PASS] Start Pose is collision-free." << std::endl;

  // 2. Check Goal Pose Validity
  Corners goal_c =
      get_corners(goal.x, goal.y, goal.yaw, robot->size.front_length,
                  robot->size.rear_length, robot->size.width);
  bool goal_ok = true;
  for (const auto &[ent, pose] : others)
  {
    if (ent == robot)
      continue;

    Corners ent_c =
        get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length,
                    ent->size.rear_length, ent->size.width);
    if (rectangles_intersect(goal_c, ent_c))
    {
      std::cerr << "    [FAIL] Goal Pose COLLIDES with " << ent->name
                << std::endl;
      goal_ok = false;
    }
  }
  if (goal_ok)
    std::cerr << "    [PASS] Goal Pose is collision-free." << std::endl;

  // 3. Check Global Consistency
  std::cerr << "    [Info] Checking other entities for consistency..."
            << std::endl;
  bool global_issue = false;

  for (size_t i = 0; i < others.size(); ++i)
  {
    for (size_t j = i + 1; j < others.size(); ++j)
    {
      auto [ent1, p1] = others[i];
      auto [ent2, p2] = others[j];

      if (ent1 == robot || ent2 == robot)
        continue;

      Corners c1 = get_corners(p1.x, p1.y, p1.yaw, ent1->size.front_length,
                               ent1->size.rear_length, ent1->size.width);
      Corners c2 = get_corners(p2.x, p2.y, p2.yaw, ent2->size.front_length,
                               ent2->size.rear_length, ent2->size.width);

      if (rectangles_intersect(c1, c2))
      {
        // Check if this is a valid transfer (Robot pushing Object)
        if (is_valid_transfer(ent1, p1, ent2, p2))
        {
          // Valid transfer - ignore
          // std::cout << "    [Info] Ignoring contact between " << ent1->name
          // << " and " << ent2->name << " (Transferring)" << std::endl;
        }
        else
        {
          std::cerr << "    [WARN] Global Consistency: " << ent1->name
                    << " intersects " << ent2->name << "!" << std::endl;
          global_issue = true;
        }
      }
    }
  }
  if (!global_issue)
    std::cerr << "    [PASS] Global scene is consistent (ignoring transfers)."
              << std::endl;
}

// ==========================================
// 2. CORE PLANNING SUB-ROUTINES (Moved up)
// ==========================================

const double INF = std::numeric_limits<double>::infinity();

std::vector<ParkingCandidate>
generate_parking_candidates_randomized(const Pose &current_pose,
                                       RobotMeta *robot,
                                       const Params &params)
{
  std::vector<ParkingCandidate> candidates;

  double maxc = 1.0 / std::max(robot->transit_turning_radius(), 1e-6);
  double step_size = 0.2; // Can be coarse; only used for discretization (we
                          // ignore the full path)
  double wb = robot->wheel_base;

  auto compute_rs_length = [&](const Pose &goal) -> double
  {
    auto [xs, ys, yaws, ctypes, lengths, steers, directions] =
        ReedShepp::reeds_shepp_path_planning(current_pose.x, current_pose.y,
                                             current_pose.yaw, goal.x, goal.y,
                                             goal.yaw, maxc, step_size, wb);

    if (xs.empty())
    {
      return INF;
    }
    double L = 0.0;
    for (double len : lengths)
    {
      L += std::abs(len);
    }
    return L;
  };

  // 1. Strongly prefer the robot's initial_pose (often designed to be
  // safe/clear)
  double init_L = compute_rs_length(robot->initial_pose);
  candidates.push_back({robot->initial_pose, init_L});

  // 2. Workspace corners with multiple orientations (out-of-the-way locations)
  double margin = 0.6; // Safe margin from exact bounds
  std::vector<double> corner_yaws = {0.0, M_PI / 2, M_PI, -M_PI / 2};
  std::vector<std::pair<double, double>> corner_positions = {
      {params.min_x + margin, params.min_y + margin},
      {params.max_x - margin, params.min_y + margin},
      {params.max_x - margin, params.max_y - margin},
      {params.min_x + margin, params.max_y - margin}};

  for (const auto &pos : corner_positions)
  {
    for (double yaw : corner_yaws)
    {
      Pose p{pos.first, pos.second, yaw};
      double L = compute_rs_length(p);
      candidates.push_back({p, L});
    }
  }

  // 2.5. Nearby ring samples (prefer short clear-away moves)
  std::vector<double> ring_r = {0.6, 0.9, 1.2};
  std::vector<double> ring_a = {0.0, M_PI / 4, M_PI / 2, 3 * M_PI / 4,
                                M_PI, -3 * M_PI / 4, -M_PI / 2, -M_PI / 4};
  for (double r : ring_r)
  {
    for (double a : ring_a)
    {
      Pose p{current_pose.x + r * std::cos(a),
             current_pose.y + r * std::sin(a),
             a};
      if (p.x < params.min_x + margin || p.x > params.max_x - margin ||
          p.y < params.min_y + margin || p.y > params.max_y - margin)
      {
        continue;
      }
      double L = compute_rs_length(p);
      candidates.push_back({p, L});
    }
  }

  // 3. Random samples as fallback (20-30 is plenty)
  auto &gen = parking_rng();
  std::uniform_real_distribution<> x_dist(params.min_x + margin,
                                          params.max_x - margin);
  std::uniform_real_distribution<> y_dist(params.min_y + margin,
                                          params.max_y - margin);
  std::uniform_real_distribution<> yaw_dist(-M_PI, M_PI);

  for (int i = 0; i < 25; ++i)
  {
    Pose p{x_dist(gen), y_dist(gen), yaw_dist(gen)};
    double L = compute_rs_length(p);
    candidates.push_back({p, L});
  }

  // Sort by Reeds-Shepp length (lower is better). INF goes to the end.
  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b)
            {
              constexpr double eps = 1e-6;
              if (std::abs(a.estimated_rs_length - b.estimated_rs_length) > eps)
              {
                return a.estimated_rs_length < b.estimated_rs_length;
              }
              if (std::abs(a.pose.x - b.pose.x) > eps)
                return a.pose.x < b.pose.x;
              if (std::abs(a.pose.y - b.pose.y) > eps)
                return a.pose.y < b.pose.y;
              return a.pose.yaw < b.pose.yaw;
            });
  return candidates;
}

Pose propagate_pose_with_primitive(const Pose &pose,
                                   int direction,
                                   double steer,
                                   double speed,
                                   double wheel_base,
                                   double dt)
{
  Pose out = pose;
  double distance = static_cast<double>(direction) * speed * dt;
  if (std::abs(distance) < 1e-9)
  {
    return out;
  }

  double curvature = std::tan(steer) / std::max(wheel_base, 1e-6);
  if (std::abs(curvature) < 1e-9)
  {
    out.x += distance * std::cos(pose.yaw);
    out.y += distance * std::sin(pose.yaw);
    out.yaw = mod2pi(pose.yaw);
    return out;
  }

  double yaw_delta = distance * curvature;
  double radius = 1.0 / curvature;
  out.x += radius * (std::sin(pose.yaw + yaw_delta) - std::sin(pose.yaw));
  out.y += -radius * (std::cos(pose.yaw + yaw_delta) - std::cos(pose.yaw));
  out.yaw = mod2pi(pose.yaw + yaw_delta);
  return out;
}

std::vector<ParkingCandidate>
generate_parking_candidates_expand_primitives(const Pose &current_pose,
                                              RobotMeta *robot,
                                              const Params &params)
{
  std::vector<ParkingCandidate> candidates;
  if (!robot)
    return candidates;

  constexpr double kDeltaT = 0.5;
  constexpr int kMaxPrimitiveDepth = 12;
  constexpr size_t kMaxCandidates = 300;
  constexpr double kMinParkingDisplacement = 0.25;

  const double max_curvature =
      1.0 / std::max(robot->transit_turning_radius(), 1e-6);
  const double max_steer = std::atan(robot->wheel_base * max_curvature);
  const std::vector<std::pair<int, double>> primitives = {
      {1, 0.0},
      {1, max_steer / 2.0},
      {1, -max_steer / 2.0},
      {1, max_steer},
      {1, -max_steer},
      {-1, 0.0},
      {-1, max_steer / 2.0},
      {-1, -max_steer / 2.0},
      {-1, max_steer},
      {-1, -max_steer}};

  struct ExpansionNode
  {
    Pose pose;
    int depth = 0;
  };

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::vector<ExpansionNode> frontier;
  frontier.push_back({current_pose, 0});

  std::unordered_set<std::string> visited;
  visited.insert(pose_key(current_pose));

  for (int depth = 1; depth <= kMaxPrimitiveDepth && candidates.size() < kMaxCandidates; ++depth)
  {
    std::vector<ExpansionNode> next_frontier;
    for (const auto &node : frontier)
    {
      for (const auto &[direction, steer] : primitives)
      {
        Pose next_pose = propagate_pose_with_primitive(
            node.pose, direction, steer,
            robot->speed_transit,
            robot->wheel_base,
            kDeltaT);

        std::string key = pose_key(next_pose);
        if (visited.count(key))
          continue;
        visited.insert(key);

        CollisionGeometry geom_bounds = setup_collision_geometry(next_pose, robot->size, 1.0);
        if (check_robot_bounds_collision(next_pose, geom_bounds.corners, params))
          continue;

        next_frontier.push_back({next_pose, depth});

        double displacement = std::hypot(next_pose.x - current_pose.x,
                                         next_pose.y - current_pose.y);
        if (displacement < kMinParkingDisplacement)
          continue;

        double yaw_delta = std::atan2(std::sin(next_pose.yaw - current_pose.yaw),
                                      std::cos(next_pose.yaw - current_pose.yaw));
        double travel_cost = depth * std::max(robot->speed_transit, 1e-6) * kDeltaT;
        double estimated_cost = travel_cost + 0.15 * std::abs(yaw_delta);
        candidates.push_back({next_pose, estimated_cost});

        if (candidates.size() >= kMaxCandidates)
          break;
      }
      if (candidates.size() >= kMaxCandidates)
        break;
    }

    if (next_frontier.empty())
      break;
    frontier = std::move(next_frontier);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b)
            {
              constexpr double eps = 1e-6;
              if (std::abs(a.estimated_rs_length - b.estimated_rs_length) > eps)
              {
                return a.estimated_rs_length < b.estimated_rs_length;
              }
              if (std::abs(a.pose.x - b.pose.x) > eps)
                return a.pose.x < b.pose.x;
              if (std::abs(a.pose.y - b.pose.y) > eps)
                return a.pose.y < b.pose.y;
              return a.pose.yaw < b.pose.yaw;
            });

  return candidates;
}

std::vector<ParkingCandidate>
generate_parking_candidates_connected_primitives(
    const Pose &current_pose,
    RobotMeta *robot,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    double start_time)
{
  std::vector<ParkingCandidate> candidates;
  if (!robot)
    return candidates;

  constexpr int kMaxExpandIterations = 3000;
  constexpr int kMaxPrimitiveDepth = 12;
  constexpr size_t kMaxCandidates = 200;
  constexpr double kMinParkingDisplacement = 0.25;

  PHAStar planner(robot, current_pose, &timetable, &entities, params, false,
                  "", start_time, "Safe parking connected primitives");
  planner.set_ignore_other_robots(true);
  planner.set_debug_popup_enabled(false);

  using PQElem = std::tuple<double, std::uint64_t, Node *, int>;
  auto cmp = [](const PQElem &a, const PQElem &b)
  {
    return std::get<0>(a) > std::get<0>(b) ||
           (std::get<0>(a) == std::get<0>(b) &&
            std::get<1>(a) > std::get<1>(b));
  };

  std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
  std::uint64_t node_id = 0;

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::unordered_map<std::string, double> best_cost_by_pose;
  std::unordered_set<std::string> emitted_candidate_keys;
  best_cost_by_pose.emplace(pose_key(current_pose), 0.0);
  open_set.emplace(0.0, node_id++, planner.start.get(), 0);

  int iterations = 0;
  while (!open_set.empty() &&
         iterations < kMaxExpandIterations &&
         candidates.size() < kMaxCandidates)
  {
    auto [priority_cost, _, current, depth] = open_set.top();
    open_set.pop();
    ++iterations;

    if (depth > kMaxPrimitiveDepth)
      continue;

    Pose current_node_pose{current->x, current->y, current->yaw};
    std::string current_key = pose_key(current_node_pose);
    auto best_it = best_cost_by_pose.find(current_key);
    if (best_it != best_cost_by_pose.end() &&
        priority_cost > best_it->second + 1e-9)
    {
      continue;
    }

    if (depth > 0)
    {
      double displacement = std::hypot(current->x - current_pose.x,
                                       current->y - current_pose.y);
      if (displacement >= kMinParkingDisplacement &&
          emitted_candidate_keys.insert(current_key).second)
      {
        ParkingCandidate candidate;
        candidate.pose = current_node_pose;
        candidate.estimated_rs_length = current->cost;
        candidate.connected_waypoints =
            planner.extract_path(
                current, std::vector<std::tuple<double, double, double>>{});
        candidates.push_back(std::move(candidate));
      }
    }

    if (depth >= kMaxPrimitiveDepth)
      continue;

    for (const auto &prim : planner.motion_primitives)
    {
      if (prim.first == 0)
        continue;

      Node *new_node = planner.generate_node(current, prim);
      if (!new_node)
        continue;
      if (!planner.check_collision_along_path(current, new_node, prim))
        continue;

      Pose next_pose{new_node->x, new_node->y, new_node->yaw};
      std::string next_key = pose_key(next_pose);
      auto cost_it = best_cost_by_pose.find(next_key);
      if (cost_it != best_cost_by_pose.end() &&
          new_node->cost + 1e-9 >= cost_it->second)
      {
        continue;
      }

      best_cost_by_pose[next_key] = new_node->cost;
      open_set.emplace(new_node->cost, node_id++, new_node, depth + 1);
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b)
            {
              constexpr double eps = 1e-6;
              if (std::abs(a.estimated_rs_length - b.estimated_rs_length) > eps)
              {
                return a.estimated_rs_length < b.estimated_rs_length;
              }
              if (std::abs(a.pose.x - b.pose.x) > eps)
                return a.pose.x < b.pose.x;
              if (std::abs(a.pose.y - b.pose.y) > eps)
                return a.pose.y < b.pose.y;
              return a.pose.yaw < b.pose.yaw;
            });

  return candidates;
}

std::vector<ParkingCandidate>
generate_parking_candidates_reverse_recent_path(
    const Pose &current_pose,
    RobotMeta *robot,
    TimeTable &timetable,
    double start_time)
{
  std::vector<ParkingCandidate> candidates;
  if (!robot)
    return candidates;

  constexpr double kMinParkingDisplacement = 0.25;
  constexpr std::size_t kMaxCandidates = 200;
  constexpr std::size_t kMaxReverseStates = 400;

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  auto poses_match = [](const Pose &a, const Pose &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y) <= 1e-4 &&
           std::abs(pi_2_pi(a.yaw - b.yaw)) <= 1e-3;
  };

  const auto &database = timetable.get_database();
  auto db_it = database.find(robot);
  if (db_it == database.end() || db_it->second.size() < 2)
    return candidates;

  const double max_gap = std::max(0.75, timetable.time_increment * 1.5);
  const auto &timeline = db_it->second;
  const auto &spans = timetable.get_trajectory_spans();

  auto find_latest_span_before = [&]() -> std::pair<const TimeTable::TrajectorySpan *, std::size_t>
  {
    const TimeTable::TrajectorySpan *latest_span = nullptr;
    std::size_t latest_span_index = 0;
    for (std::size_t i = 0; i < spans.size(); ++i)
    {
      const auto &span = spans[i];
      if (span.entity != robot)
        continue;
      if (span.end_time > start_time + 1e-6)
        continue;
      if (!latest_span || span.end_time > latest_span->end_time)
      {
        latest_span = &span;
        latest_span_index = i;
      }
    }
    return {latest_span, latest_span_index};
  };

  auto find_previous_non_retraction_span =
      [&](std::size_t before_index) -> const TimeTable::TrajectorySpan *
  {
    for (std::size_t i = before_index; i-- > 0;)
    {
      if (spans[i].entity != robot)
        continue;
      if (spans[i].kind != TrajectoryKind::RETRACTION)
        return &spans[i];
    }
    return nullptr;
  };

  auto collect_span_states =
      [&](double span_start, double span_end,
          bool exclude_terminal) -> std::vector<std::pair<double, Pose>>
  {
    std::vector<std::pair<double, Pose>> states;
    double query_end = span_end + 1e-6;
    if (exclude_terminal)
      query_end = span_end - 1e-6;

    auto latest_after = timeline.upper_bound(query_end);
    if (latest_after == timeline.begin())
      return states;

    auto latest_it = std::prev(latest_after);
    if (latest_it->first < span_start - 1e-6)
      return states;

    states.reserve(std::min<std::size_t>(timeline.size(), kMaxReverseStates));
    states.push_back(*latest_it);

    auto it = latest_it;
    while (it != timeline.begin() && states.size() < kMaxReverseStates)
    {
      auto older_it = std::prev(it);
      if (older_it->first < span_start - 1e-6)
        break;
      if ((it->first - older_it->first) > max_gap)
        break;
      states.push_back(*older_it);
      it = older_it;
    }
    return states;
  };

  auto [latest_span, latest_span_index] = find_latest_span_before();
  if (!latest_span)
    return candidates;

  double source_start_time = latest_span->start_time;
  double source_end_time = latest_span->end_time;
  bool use_connected_waypoints = latest_span->kind == TrajectoryKind::TRANSIT;
  bool exclude_terminal_state = latest_span->kind != TrajectoryKind::TRANSIT;

  if (latest_span->kind == TrajectoryKind::RETRACTION)
  {
    const auto *previous_non_retraction =
        find_previous_non_retraction_span(latest_span_index);
    if (!previous_non_retraction)
      return candidates;
    source_start_time = previous_non_retraction->start_time;
    source_end_time = previous_non_retraction->end_time;
    use_connected_waypoints = false;
    exclude_terminal_state = true;
  }

  if (use_connected_waypoints)
  {
    std::size_t scan_index = latest_span_index;
    while (scan_index > 0)
    {
      const TimeTable::TrajectorySpan *previous_span = nullptr;
      std::size_t previous_index = 0;
      for (std::size_t i = scan_index; i-- > 0;)
      {
        if (spans[i].entity == robot)
        {
          previous_span = &spans[i];
          previous_index = i;
          break;
        }
      }

      if (!previous_span || previous_span->kind != TrajectoryKind::TRANSIT)
        break;
      if ((source_start_time - previous_span->end_time) > max_gap)
        break;

      source_start_time = previous_span->start_time;
      scan_index = previous_index;
    }
  }

  std::vector<std::pair<double, Pose>> source_states =
      collect_span_states(source_start_time, source_end_time,
                          exclude_terminal_state);

  if (source_states.size() < 2)
    return candidates;

  if (!use_connected_waypoints)
  {
    std::unordered_set<std::string> emitted_candidate_keys;
    for (const auto &[source_time, source_pose] : source_states)
    {
      const Pose candidate_pose{source_pose.x, source_pose.y, source_pose.yaw};
      const double displacement = std::hypot(candidate_pose.x - current_pose.x,
                                             candidate_pose.y - current_pose.y);
      if (displacement < kMinParkingDisplacement)
        continue;
      if (!emitted_candidate_keys.insert(pose_key(candidate_pose)).second)
        continue;

      ParkingCandidate candidate;
      candidate.pose = candidate_pose;
      candidate.estimated_rs_length = displacement;
      candidates.push_back(std::move(candidate));
      if (candidates.size() >= kMaxCandidates)
        break;
    }

    return candidates;
  }

  std::vector<Waypoint> reverse_path;
  reverse_path.reserve(source_states.size() + 1);

  Waypoint start_wp;
  start_wp.x = current_pose.x;
  start_wp.y = current_pose.y;
  start_wp.yaw = current_pose.yaw;
  start_wp.time = start_time;
  reverse_path.push_back(start_wp);

  std::unordered_set<std::string> emitted_candidate_keys;
  double candidate_time = start_time;
  double previous_source_time = source_states.front().first;
  Pose last_added_pose = current_pose;

  for (const auto &[source_time, source_pose] : source_states)
  {
    if (poses_match(last_added_pose, source_pose))
    {
      previous_source_time = source_time;
      continue;
    }

    const double travel_dt = std::max(1e-3, previous_source_time - source_time);
    candidate_time += travel_dt;

    Waypoint wp;
    wp.x = source_pose.x;
    wp.y = source_pose.y;
    wp.yaw = source_pose.yaw;
    wp.time = candidate_time;
    reverse_path.push_back(wp);

    previous_source_time = source_time;
    last_added_pose = source_pose;
  }

  if (reverse_path.size() < 2)
    return candidates;

  for (std::size_t i = 1; i < reverse_path.size() && candidates.size() < kMaxCandidates; ++i)
  {
    const Pose candidate_pose{reverse_path[i].x, reverse_path[i].y, reverse_path[i].yaw};
    const double displacement = std::hypot(candidate_pose.x - current_pose.x,
                                           candidate_pose.y - current_pose.y);
    if (displacement < kMinParkingDisplacement)
      continue;

    if (!emitted_candidate_keys.insert(pose_key(candidate_pose)).second)
      continue;

    ParkingCandidate candidate;
    candidate.pose = candidate_pose;
    candidate.estimated_rs_length = reverse_path[i].time - start_time;
    candidate.connected_waypoints.assign(reverse_path.begin(),
                                         reverse_path.begin() + static_cast<std::ptrdiff_t>(i + 1));
    candidates.push_back(std::move(candidate));
  }

  return candidates;
}

std::vector<ParkingCandidate>
generate_parking_candidates(const Pose &current_pose, RobotMeta *robot,
                            const Params &params,
                            TimeTable *timetable = nullptr,
                            const std::unordered_map<std::string, EntityMeta *> *entities = nullptr,
                            double start_time = 0.0)
{
  if (g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT ||
      g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
  {
    if (!timetable)
      return {};
    return generate_parking_candidates_reverse_recent_path(current_pose, robot,
                                                           *timetable, start_time);
  }
  if (g_parking_candidate_mode == ParkingCandidateMode::CONNECTED ||
      g_parking_candidate_mode == ParkingCandidateMode::CONNECTED_VFH)
  {
    if (!timetable || !entities)
      return {};
    return generate_parking_candidates_connected_primitives(current_pose, robot,
                                                            *timetable, *entities,
                                                            params, start_time);
  }
  if (g_parking_candidate_mode == ParkingCandidateMode::EXPAND)
  {
    return generate_parking_candidates_expand_primitives(current_pose, robot,
                                                         params);
  }
  return generate_parking_candidates_randomized(current_pose, robot, params);
}

std::vector<ParkingCandidate>
generate_parking_candidates_for_mode(const Pose &current_pose,
                                     RobotMeta *robot,
                                     const Params &params,
                                     ParkingCandidateMode mode,
                                     TimeTable *timetable = nullptr,
                                     const std::unordered_map<std::string, EntityMeta *> *entities = nullptr,
                                     double start_time = 0.0)
{
  if (mode == ParkingCandidateMode::REVERSE_RECENT ||
      mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
  {
    if (!timetable)
      return {};
    return generate_parking_candidates_reverse_recent_path(current_pose, robot,
                                                           *timetable, start_time);
  }
  if (mode == ParkingCandidateMode::CONNECTED ||
      mode == ParkingCandidateMode::CONNECTED_VFH)
  {
    if (!timetable || !entities)
      return {};
    return generate_parking_candidates_connected_primitives(current_pose, robot,
                                                            *timetable, *entities,
                                                            params, start_time);
  }
  if (mode == ParkingCandidateMode::EXPAND)
  {
    return generate_parking_candidates_expand_primitives(current_pose, robot,
                                                         params);
  }
  return generate_parking_candidates_randomized(current_pose, robot, params);
}

bool is_pose_collision_free_at_time(EntityMeta *entity, const Pose &pose,
                                    double t, TimeTable &timetable,
                                    const Params &params)
{
  if (!entity)
    return false;

  CollisionGeometry geom = setup_collision_geometry_for_type(
      pose,
      entity->type,
      entity->size,
      params);
  CollisionGeometry geom_bounds = setup_collision_geometry(pose, entity->size, 1.0);
  bool out_of_bounds = check_entity_bounds_collision(
      entity->type,
      pose,
      geom_bounds.corners,
      params);

  if (out_of_bounds)
  {
    return false;
  }

  auto poses = timetable.get_poses(t);
  for (const auto &[ent, ent_pose] : poses)
  {
    if (ent == entity)
      continue;
    auto c = check_entity_collision(geom, pose, ent, ent_pose, params);
    if (c.has_collision)
      return false;
  }
  return true;
}

bool is_parking_pose_safe_until_last_timestamp(EntityMeta *entity,
                                               const Pose &pose,
                                               double from_t,
                                               TimeTable &timetable,
                                               const Params &params,
                                               double step = 0.5)
{
  double max_t = timetable.get_max_time();
  for (double t = from_t; t <= max_t + 1e-9; t += step)
  {
    if (!is_pose_collision_free_at_time(entity, pose, t, timetable, params))
      return false;
  }
  return true;
}

void project_waypoints_inside_bounds(std::vector<Waypoint> &waypoints,
                                     RobotMeta *robot,
                                     const Params &params)
{
  if (!robot)
    return;

  for (auto &wp : waypoints)
  {
    Corners corners = get_corners(wp.x, wp.y, wp.yaw,
                                  robot->size.front_length,
                                  robot->size.rear_length,
                                  robot->size.width);

    double min_cx = corners[0].x, max_cx = corners[0].x;
    double min_cy = corners[0].y, max_cy = corners[0].y;
    for (const auto &pt : corners)
    {
      min_cx = std::min(min_cx, pt.x);
      max_cx = std::max(max_cx, pt.x);
      min_cy = std::min(min_cy, pt.y);
      max_cy = std::max(max_cy, pt.y);
    }

    double dx = 0.0;
    if (min_cx < params.min_x)
      dx += (params.min_x - min_cx);
    if (max_cx > params.max_x)
      dx += (params.max_x - max_cx);

    double dy = 0.0;
    if (min_cy < params.min_y)
      dy += (params.min_y - min_cy);
    if (max_cy > params.max_y)
      dy += (params.max_y - max_cy);

    wp.x += dx;
    wp.y += dy;
  }
}

void shift_waypoint_times(std::vector<Waypoint> &waypoints, double delta)
{
  if (std::abs(delta) < 1e-12)
    return;

  for (auto &wp : waypoints)
    wp.time += delta;
}

void make_waypoint_times_relative(std::vector<Waypoint> &waypoints,
                                  double reference_time)
{
  shift_waypoint_times(waypoints, -reference_time);
}

Params make_relaxed_fallback_params(const Params &params)
{
  Params relaxed = params;
  relaxed.xy_resolution = std::max(params.xy_resolution, 0.3);
  relaxed.yaw_resolution = std::max(params.yaw_resolution, M_PI / 2.0);
  relaxed.time_step = std::max(params.time_step, 3.0);
  relaxed.time_resolution = relaxed.time_step;
  relaxed.collision_steps = std::max(2, params.collision_steps);
  relaxed.rs_step_size = std::max(params.rs_step_size, 0.3);
  return relaxed;
}

Params make_fine_segment_params(const Params &params,
                                const RuntimeOptions &options)
{
  Params fine = params;
  fine.xy_resolution = std::min(params.xy_resolution,
                                std::max(1e-6, options.fine_segment_xy_resolution));
  fine.yaw_resolution = std::min(params.yaw_resolution,
                                 std::max(1e-6, options.fine_segment_yaw_resolution));
  fine.time_step = std::min(params.time_step,
                            std::max(1e-6, options.fine_segment_time_step));
  fine.time_resolution = fine.time_step;
  fine.rs_step_size = std::min(params.rs_step_size,
                               std::max(1e-6, options.fine_segment_rs_step_size));
  fine.collision_check_time_step =
      std::min(params.collision_check_time_step,
               std::max(1e-6, options.fine_segment_collision_check_time_step));
  fine.collision_steps = std::max(
      params.collision_steps,
      static_cast<int>(std::ceil(fine.time_step /
                                 std::max(1e-3, fine.collision_check_time_step))));
  return fine;
}

ReferenceExperienceGraphOptions reference_egraph_options_from_runtime(
    const RuntimeOptions &options)
{
  ReferenceExperienceGraphOptions egraph_options;
  egraph_options.enabled = options.enable_reference_egraph_transit;
  egraph_options.epsilon = options.reference_egraph_epsilon;
  egraph_options.waypoint_spacing = options.reference_egraph_waypoint_spacing;
  egraph_options.snap_radius = options.reference_egraph_snap_radius;
  egraph_options.snap_yaw = options.reference_egraph_snap_yaw;
  egraph_options.successor_lookahead =
      options.reference_egraph_successor_lookahead;
  egraph_options.max_nodes = options.reference_egraph_max_nodes;
  return egraph_options;
}

std::vector<Waypoint> waypoints_from_relopush_state_path(
    const ReloPush::StatePathPtr &path)
{
  std::vector<Waypoint> waypoints;
  if (!path)
    return waypoints;

  waypoints.reserve(path->size());
  for (const auto &state : *path)
    waypoints.push_back(WaypointFromReloPushState(state));
  return waypoints;
}

void accumulate_wait_stats(TaskExecutionStats *stats, double wait_added)
{
  if (stats && wait_added > 1e-9)
  {
    stats->total_waiting += wait_added;
    stats->delayed_segments += 1;
  }
}

void append_transfer_window_if_needed(
    const Trajectory &traj,
    std::vector<TransferContactWindow> *transfer_windows)
{
  if (traj.is_transfer && traj.entity && traj.transferred_object &&
      transfer_windows && !traj.waypoints.empty())
  {
    double st = traj.start_time + traj.waypoints.front().time;
    double et = traj.start_time + traj.waypoints.back().time;
    transfer_windows->push_back({traj.entity, traj.transferred_object, st, et});
  }
}

double shared_collision_check_step(const Params &params)
{
  const double planner_step =
      params.time_step / static_cast<double>(std::max(1, params.collision_steps));
  // Use a finer shared replay step so scheduling and post-validation sample the
  // same motion densely enough to catch slight transient overlaps.
  return std::max(1e-3, std::min(params.collision_check_time_step,
                                 planner_step));
}

enum class IdleBlockerRelocationPolicy
{
  RelocateAnyIdle,
  WaitOnly,
  RelocateIfBecameIdleDuringAttempt,
};

// ==========================================
// Collision check helpers
// ==========================================
CollisionInfo check_collision_trajectory_detailed(const Trajectory &traj, double start_time,
                                                  TimeTable &timetable, const Params &params,
                                                  bool verbose);

CollisionInfo check_collision_trajectory_against_entity(
    const Trajectory &traj,
    double start_time,
    EntityMeta *other_entity,
    TimeTable &timetable,
    const Params &params);

double find_wait_only_start_time_avoiding_entity(
    const Trajectory &traj,
    double earliest_start,
    EntityMeta *blocking_entity,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

double find_wait_only_start_time(
    const Trajectory &traj,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

double find_safe_start_time(Trajectory *traj, double earliest_start,
                            TimeTable &timetable, const Params &params,
                            const std::unordered_map<std::string, EntityMeta *> &entities,
                            const RuntimeOptions &options,
                            double *out_wait_added,
                            CollisionInfo *out_last_collision = nullptr,
                            double *out_last_check_time = nullptr,
                            IdleBlockerRelocationPolicy idle_blocker_policy =
                                IdleBlockerRelocationPolicy::RelocateAnyIdle);

double timetable_delay_search_horizon(double earliest_start,
                                      const TimeTable &timetable,
                                      double step)
{
  return std::max(earliest_start, timetable.get_max_time()) +
         std::max(step, 1e-3);
}

bool check_collision_trajectory(const Trajectory &traj, double start_time,
                                TimeTable &timetable, const Params &params,
                                bool verbose = false)
{
  if (traj.waypoints.empty())
    return true;
  return check_collision_trajectory_detailed(traj, start_time, timetable, params, verbose).is_valid;
}

CollisionInfo check_collision_trajectory_detailed(const Trajectory &traj, double start_time,
                                                  TimeTable &timetable, const Params &params,
                                                  bool verbose = false)
{
  if (traj.waypoints.empty())
    return {true, "Empty Trajectory", "", start_time};

  double dt = shared_collision_check_step(params);
  double duration = traj.waypoints.back().time;

  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  ObjectMeta *object = dynamic_cast<ObjectMeta *>(traj.transferred_object);

  if (!robot)
    return {false, "Invalid Trajectory", "", start_time};

  for (double t = 0; t <= duration; t += dt)
  {
    double abs_t = start_time + t;
    auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
    Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                   std::get<2>(pose_tuple)};

    CollisionGeometry r_geom = setup_collision_geometry_for_type(
        r_pose, EntityType::ROBOT, robot->size, params);

    CollisionGeometry r_geom_bounds = setup_collision_geometry(r_pose, robot->size, 1.0);

    if (check_robot_bounds_collision(r_pose, r_geom_bounds.corners, params))
    {
      return {false, "Boundary Collision", "Boundary", abs_t};
    }

    auto others = timetable.get_poses(abs_t);

    CollisionGeometry obj_geom;
    Pose obj_pose;
    const CollisionGeometry *obj_geom_ptr = nullptr;
    const Pose *obj_pose_ptr = nullptr;

    if (traj.is_transfer && object)
    {
      double offset = robot->size.front_length + object->size.rear_length;
      obj_pose = {r_pose.x + offset * std::cos(r_pose.yaw),
                  r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
      obj_geom = setup_collision_geometry_for_type(
          obj_pose, EntityType::OBJECT, object->size, params);

      CollisionGeometry obj_geom_bounds = setup_collision_geometry(obj_pose, object->size, 1.0);

      if (check_object_bounds_collision(obj_pose, obj_geom_bounds.corners, params))
      {
        return {false, "Boundary Collision", "Boundary", abs_t};
      }

      obj_geom_ptr = &obj_geom;
      obj_pose_ptr = &obj_pose;
    }

    auto collision_result = check_multiple_entities_collision(
        r_geom, r_pose,
        obj_geom_ptr, obj_pose_ptr,
        others,
        params,
        robot,
        object,
        nullptr);
    if (collision_result.has_collision)
    {
      if (t <= 0.3 && collision_result.colliding_entity &&
          collision_result.colliding_entity->type == EntityType::OBJECT)
      {
        auto it = others.find(collision_result.colliding_entity);
        if (it != others.end() && is_valid_transfer_contact(robot, r_pose, it->first, it->second))
        {
          continue;
        }
      }

      return {false, collision_result.collision_type + " Collision",
              collision_result.colliding_entity ? collision_result.colliding_entity->name : "",
              abs_t};
    }
  }
  return {true, "Valid", "", 0.0};
}

CollisionInfo check_collision_trajectory_against_entity(
    const Trajectory &traj,
    double start_time,
    EntityMeta *other_entity,
    TimeTable &timetable,
    const Params &params)
{
  if (traj.waypoints.empty() || !other_entity)
    return {true, "Valid", "", start_time};

  double dt = shared_collision_check_step(params);
  double duration = traj.waypoints.back().time;

  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  ObjectMeta *object = dynamic_cast<ObjectMeta *>(traj.transferred_object);

  if (!robot)
    return {false, "Invalid Trajectory", "", start_time};

  for (double t = 0; t <= duration + 1e-9; t += dt)
  {
    double abs_t = start_time + t;
    auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
    Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                   std::get<2>(pose_tuple)};

    CollisionGeometry r_geom = setup_collision_geometry_for_type(
        r_pose, EntityType::ROBOT, robot->size, params);

    CollisionGeometry obj_geom;
    Pose obj_pose;
    const CollisionGeometry *obj_geom_ptr = nullptr;
    const Pose *obj_pose_ptr = nullptr;

    if (traj.is_transfer && object)
    {
      double offset = robot->size.front_length + object->size.rear_length;
      obj_pose = {r_pose.x + offset * std::cos(r_pose.yaw),
                  r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
      obj_geom = setup_collision_geometry_for_type(
          obj_pose, EntityType::OBJECT, object->size, params);
      obj_geom_ptr = &obj_geom;
      obj_pose_ptr = &obj_pose;
    }

    std::unordered_map<EntityMeta *, Pose> other_pose_map;
    other_pose_map[other_entity] = timetable.get_pose(other_entity, abs_t);

    auto collision_result = check_multiple_entities_collision(
        r_geom, r_pose,
        obj_geom_ptr, obj_pose_ptr,
        other_pose_map,
        params,
        robot,
        object,
        nullptr);

    if (collision_result.has_collision)
    {
      if (t <= 0.3 && collision_result.colliding_entity &&
          collision_result.colliding_entity->type == EntityType::OBJECT)
      {
        auto it = other_pose_map.find(collision_result.colliding_entity);
        if (it != other_pose_map.end() &&
            is_valid_transfer_contact(robot, r_pose, it->first, it->second))
        {
          continue;
        }
      }

      return {false,
              collision_result.collision_type + " Collision",
              collision_result.colliding_entity ? collision_result.colliding_entity->name : "",
              abs_t};
    }
  }

  return {true, "Valid", "", 0.0};
}

double find_wait_only_start_time_avoiding_entity(
    const Trajectory &traj,
    double earliest_start,
    EntityMeta *blocking_entity,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision,
    double *out_last_check_time)
{
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  double check_time = earliest_start;
  constexpr double step = 0.5;
  CollisionInfo last_collision{true, "Not evaluated", "", earliest_start};
  double last_check_time = earliest_start;

  while (check_time <=
         timetable_delay_search_horizon(earliest_start, timetable, step) +
             1e-9)
  {
    CollisionInfo col_info = check_collision_trajectory_against_entity(
        traj, check_time, blocking_entity, timetable, params);
    last_collision = col_info;
    last_check_time = check_time;

    if (col_info.is_valid)
    {
      if (out_last_collision)
        *out_last_collision = col_info;
      if (out_last_check_time)
        *out_last_check_time = check_time;
      return check_time;
    }

    check_time += step;
  }

  if (out_last_collision)
    *out_last_collision = last_collision;
  if (out_last_check_time)
    *out_last_check_time = last_check_time;
  return -1.0;
}

double find_wait_only_start_time(
    const Trajectory &traj,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision,
    double *out_last_check_time)
{
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  double check_time = earliest_start;
  constexpr double step = 0.5;
  CollisionInfo last_collision{true, "Not evaluated", "", earliest_start};
  double last_check_time = earliest_start;

  while (check_time <=
         timetable_delay_search_horizon(earliest_start, timetable, step) +
             1e-9)
  {
    CollisionInfo col_info =
        check_collision_trajectory_detailed(traj, check_time, timetable, params, false);
    last_collision = col_info;
    last_check_time = check_time;

    if (col_info.is_valid)
    {
      if (out_last_collision)
        *out_last_collision = col_info;
      if (out_last_check_time)
        *out_last_check_time = check_time;
      return check_time;
    }

    check_time += step;
  }

  if (out_last_collision)
    *out_last_collision = last_collision;
  if (out_last_check_time)
    *out_last_check_time = last_check_time;
  return -1.0;
}

bool parking_pose_conflicts_with_blocked_hint(const Pose &candidate_pose,
                                              RobotMeta *blocker,
                                              const Trajectory *blocked_traj_hint)
{
  if (!blocker || !blocked_traj_hint)
    return false;

  Corners cand_corners = get_corners(candidate_pose.x, candidate_pose.y, candidate_pose.yaw,
                                     blocker->size.front_length, blocker->size.rear_length,
                                     blocker->size.width);
  for (size_t i = 0; i < blocked_traj_hint->waypoints.size(); ++i)
  {
    const auto &wp = blocked_traj_hint->waypoints[i];
    Corners wp_corners = get_corners(wp.x, wp.y, wp.yaw,
                                     blocker->size.front_length, blocker->size.rear_length,
                                     blocker->size.width);
    if (rectangles_intersect(cand_corners, wp_corners))
    {
      return true;
    }
  }
  return false;
}

bool parking_candidate_clears_blocked_hint(
    const Trajectory *blocked_traj_hint,
    RobotMeta *blocker,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr)
{
  if (!blocked_traj_hint || !blocker || blocked_traj_hint->waypoints.empty())
  {
    if (out_last_collision)
      *out_last_collision = CollisionInfo{true, "No blocked hint", "", 0.0};
    if (out_last_check_time)
      *out_last_check_time = 0.0;
    return true;
  }

  Trajectory hint = *blocked_traj_hint;
  double hint_start_time = hint.start_time;
  if (hint_start_time <= 0.0)
  {
    hint_start_time = hint.waypoints.front().time;
    if (hint_start_time > 1e-9)
    {
      make_waypoint_times_relative(hint.waypoints, hint_start_time);
    }
  }

  double safe_start = find_wait_only_start_time_avoiding_entity(
      hint, hint_start_time, blocker, timetable, params,
      out_last_collision, out_last_check_time);
  return safe_start >= 0.0;
}

CollisionInfo check_stationary_pose_collision_at_time(EntityMeta *entity,
                                                      const Pose &pose,
                                                      double t,
                                                      TimeTable &timetable,
                                                      const Params &params)
{
  if (!entity)
    return {false, "Invalid Entity", "", t};

  CollisionGeometry geom = setup_collision_geometry_for_type(
      pose, entity->type, entity->size, params);
  CollisionGeometry geom_bounds = setup_collision_geometry(pose, entity->size, 1.0);
  if (check_entity_bounds_collision(entity->type, pose, geom_bounds.corners, params))
  {
    return {false, "Boundary Collision", "Boundary", t};
  }

  auto poses = timetable.get_poses(t);
  for (const auto &[ent, ent_pose] : poses)
  {
    if (ent == entity)
      continue;

    auto collision = check_entity_collision(geom, pose, ent, ent_pose, params);
    if (collision.has_collision)
    {
      return {false,
              std::string(ent->type == EntityType::ROBOT ? "Robot Collision"
                                                         : "Object Collision"),
              ent ? ent->name : "",
              t};
    }
  }

  return {true, "Valid", "", t};
}

CollisionInfo check_stationary_pose_collision_at_time_ignoring(
    EntityMeta *entity,
    const Pose &pose,
    double t,
    TimeTable &timetable,
    const Params &params,
    EntityMeta *ignored_entity)
{
  if (!entity)
    return {false, "Invalid Entity", "", t};

  CollisionGeometry geom = setup_collision_geometry_for_type(
      pose, entity->type, entity->size, params);
  CollisionGeometry geom_bounds = setup_collision_geometry(pose, entity->size, 1.0);
  if (check_entity_bounds_collision(entity->type, pose, geom_bounds.corners, params))
  {
    return {false, "Boundary Collision", "Boundary", t};
  }

  auto poses = timetable.get_poses(t);
  for (const auto &[ent, ent_pose] : poses)
  {
    if (ent == entity || ent == ignored_entity)
      continue;

    auto collision = check_entity_collision(geom, pose, ent, ent_pose, params);
    if (collision.has_collision)
    {
      return {false,
              std::string(ent->type == EntityType::ROBOT ? "Robot Collision"
                                                         : "Object Collision"),
              ent ? ent->name : "",
              t};
    }
  }

  return {true, "Valid", "", t};
}

CollisionInfo find_stationary_pose_conflict_until_last_timestamp(EntityMeta *entity,
                                                                 const Pose &pose,
                                                                 double from_t,
                                                                 TimeTable &timetable,
                                                                 const Params &params,
                                                                 double step = 0.5)
{
  double max_t = timetable.get_max_time();
  for (double t = from_t; t <= max_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time(entity, pose, t, timetable, params);
    if (!info.is_valid)
      return info;
  }
  return {true, "Valid", "", from_t};
}

CollisionInfo find_stationary_pose_conflict_until_last_timestamp_ignoring(
    EntityMeta *entity,
    const Pose &pose,
    double from_t,
    TimeTable &timetable,
    const Params &params,
    double step,
    EntityMeta *ignored_entity)
{
  double max_t = timetable.get_max_time();
  for (double t = from_t; t <= max_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time_ignoring(
            entity, pose, t, timetable, params, ignored_entity);
    if (!info.is_valid)
      return info;
  }
  return {true, "Valid", "", from_t};
}

CollisionInfo find_stationary_pose_conflict_over_interval(
    EntityMeta *entity,
    const Pose &pose,
    double from_t,
    double to_t,
    TimeTable &timetable,
    const Params &params,
    double step = 0.5)
{
  if (to_t <= from_t + 1e-9)
    return {true, "Valid", "", from_t};

  for (double t = from_t; t <= to_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time(entity, pose, t, timetable, params);
    if (!info.is_valid)
      return info;
  }

  return {true, "Valid", "", from_t};
}

CollisionInfo find_stationary_pose_conflict_over_interval_ignoring(
    EntityMeta *entity,
    const Pose &pose,
    double from_t,
    double to_t,
    TimeTable &timetable,
    const Params &params,
    double step,
    EntityMeta *ignored_entity)
{
  if (to_t <= from_t + 1e-9)
    return {true, "Valid", "", from_t};

  for (double t = from_t; t <= to_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time_ignoring(
            entity, pose, t, timetable, params, ignored_entity);
    if (!info.is_valid)
      return info;
  }

  return {true, "Valid", "", from_t};
}

CollisionInfo find_robot_waiting_pose_conflict_over_interval(
    RobotMeta *robot,
    const Pose &wait_pose,
    double from_t,
    double to_t,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    EntityMeta *preferred_ignored_entity = nullptr)
{
  if (!robot || to_t <= from_t + 1e-9)
    return {true, "Valid", "", from_t};

  EntityMeta *ignored_entity = preferred_ignored_entity;
  if (!ignored_entity)
  {
    std::string contact_name = find_valid_start_contact_entity(
        robot, wait_pose, from_t, timetable, entities);
    if (!contact_name.empty())
    {
      auto it = entities.find(contact_name);
      if (it != entities.end())
        ignored_entity = it->second;
    }
  }

  const double step = shared_collision_check_step(params);
  if (ignored_entity)
  {
    return find_stationary_pose_conflict_over_interval_ignoring(
        robot, wait_pose, from_t, to_t, timetable, params, step,
        ignored_entity);
  }

  return find_stationary_pose_conflict_over_interval(
      robot, wait_pose, from_t, to_t, timetable, params, step);
}

double delayed_segment_start_after_stationary_conflict(
    double current_start,
    double segment_duration,
    double conflict_time,
    double step = 0.5)
{
  const double shift_from_conflict =
      conflict_time - std::max(0.0, segment_duration) + step;
  return std::max(current_start + step, shift_from_conflict);
}

struct ConnectedSafeParkingSearchResult
{
  bool found = false;
  Pose parking_pose;
  TimeTable committed_timetable;
};

struct ParkingDirectionHistogram
{
  std::vector<double> density;
  double max_density = 0.0;

  bool empty() const
  {
    return density.empty() || max_density <= 1e-9;
  }

  double sample(double angle) const
  {
    if (density.empty())
      return 0.0;

    double wrapped = mod2pi(angle);
    double scaled = wrapped / kDubinsTwoPi * static_cast<double>(density.size());
    int idx0 = static_cast<int>(std::floor(scaled)) % static_cast<int>(density.size());
    if (idx0 < 0)
      idx0 += static_cast<int>(density.size());
    int idx1 = (idx0 + 1) % static_cast<int>(density.size());
    double frac = scaled - std::floor(scaled);
    return density[idx0] * (1.0 - frac) + density[idx1] * frac;
  }

  double sample_normalized(double angle) const
  {
    if (empty())
      return 0.0;
    return sample(angle) / max_density;
  }
};

double parking_entity_effective_radius(EntityMeta *ent, const Params &params)
{
  if (!ent)
    return 0.0;

  const double inflation = collision_inflation_for_type(ent->type, params);
  const double full_length =
      (ent->size.front_length + ent->size.rear_length) * inflation;
  const double width = ent->size.width * inflation;
  return 0.5 * std::sqrt(full_length * full_length + width * width);
}

double ray_distance_to_boundary(const Pose &origin, double angle, const Params &params)
{
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double best = std::numeric_limits<double>::infinity();

  auto update = [&](double numerator, double denom)
  {
    if (std::abs(denom) < 1e-9)
      return;
    double t = numerator / denom;
    if (t > 0.0)
      best = std::min(best, t);
  };

  update(params.min_x - origin.x, dx);
  update(params.max_x - origin.x, dx);
  update(params.min_y - origin.y, dy);
  update(params.max_y - origin.y, dy);

  return best;
}

ParkingDirectionHistogram build_parking_direction_histogram(
    RobotMeta *blocker,
    const Pose &start_pose,
    double ready_time,
    TimeTable &timetable,
    const Params &params)
{
  ParkingDirectionHistogram hist;
  constexpr int kBins = 72;
  constexpr double kEntityWeight = 1.2;
  constexpr double kBoundaryWeight = 0.8;
  constexpr double kMinClearance = 0.05;

  hist.density.assign(kBins, 0.0);
  const double angle_step = kDubinsTwoPi / static_cast<double>(kBins);
  const double blocker_radius = parking_entity_effective_radius(blocker, params);

  auto poses = timetable.get_poses(ready_time);
  for (const auto &[ent, pose] : poses)
  {
    if (!ent || ent == blocker)
      continue;

    double dx = pose.x - start_pose.x;
    double dy = pose.y - start_pose.y;
    double dist = std::hypot(dx, dy);
    if (dist < 1e-6)
    {
      for (double &v : hist.density)
        v += 10.0;
      continue;
    }

    double other_radius = parking_entity_effective_radius(ent, params);
    double blocked_radius = blocker_radius + other_radius + params.safety_margin;
    double spread = std::asin(std::min(0.999, blocked_radius / std::max(dist, blocked_radius + 1e-6)));
    spread = std::max(spread, angle_step);
    double center = std::atan2(dy, dx);
    double clearance = std::max(kMinClearance, dist - blocked_radius);
    double contribution = kEntityWeight / clearance;

    for (int bin = 0; bin < kBins; ++bin)
    {
      double bin_angle = -M_PI + (static_cast<double>(bin) + 0.5) * angle_step;
      double delta = pi_2_pi(bin_angle - center);
      double abs_delta = std::abs(delta);
      if (abs_delta > spread)
        continue;

      double taper = 1.0 - (abs_delta / spread);
      hist.density[bin] += contribution * (0.25 + 0.75 * taper);
    }
  }

  for (int bin = 0; bin < kBins; ++bin)
  {
    double bin_angle = -M_PI + (static_cast<double>(bin) + 0.5) * angle_step;
    double ray_dist = ray_distance_to_boundary(start_pose, bin_angle, params);
    if (!std::isfinite(ray_dist))
      continue;

    double clearance = std::max(kMinClearance, ray_dist - blocker_radius);
    hist.density[bin] += kBoundaryWeight / clearance;
  }

  if (!hist.density.empty())
  {
    std::vector<double> smoothed(hist.density.size(), 0.0);
    for (size_t i = 0; i < hist.density.size(); ++i)
    {
      size_t prev = (i + hist.density.size() - 1) % hist.density.size();
      size_t next = (i + 1) % hist.density.size();
      smoothed[i] = 0.2 * hist.density[prev] +
                    0.6 * hist.density[i] +
                    0.2 * hist.density[next];
    }
    hist.density = std::move(smoothed);
    hist.max_density = *std::max_element(hist.density.begin(), hist.density.end());
  }

  return hist;
}

ConnectedSafeParkingSearchResult search_safe_parking_connected_search(
    ParkingCandidateMode mode,
    RobotMeta *blocker,
    const Pose &start_pose,
    double ready_time,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Trajectory *blocked_traj_hint,
    std::vector<SafeParkingDebugTrial> *debug_trials)
{
  ConnectedSafeParkingSearchResult result;
  if (!blocker)
    return result;

  constexpr int kMaxExpandIterations = 3000;
  constexpr int kMaxPrimitiveDepth = 12;
  constexpr double kMinParkingDisplacement = 0.25;

  auto append_debug_trial = [&](const Pose &candidate_pose,
                                const PlanningResult &trial_result,
                                const TimeTable *context_timetable = nullptr)
  {
    if (!DEBUG_VIS || !debug_trials)
      return;

    SafeParkingDebugTrial trial;
    trial.trial_index = static_cast<int>(debug_trials->size()) + 1;
    trial.candidate_mode = parking_candidate_mode_name(mode);
    trial.candidate_pose = candidate_pose;
    trial.result = trial_result;
    if (blocked_traj_hint)
      trial.blocked_path_hint_waypoints = blocked_traj_hint->waypoints;
    trial.context_timetable = context_timetable ? *context_timetable : timetable;
    debug_trials->push_back(std::move(trial));
  };

  ParkingDirectionHistogram hist;
  const bool use_histogram = (mode == ParkingCandidateMode::CONNECTED_VFH);
  if (use_histogram)
  {
    hist = build_parking_direction_histogram(blocker, start_pose, ready_time,
                                             timetable, params);
  }

  const std::string debug_kind = use_histogram
                                     ? "Safe parking connected VFH search"
                                     : "Safe parking connected search";
  PHAStar planner(blocker, start_pose, &timetable, &entities, params, false,
                  "", ready_time, debug_kind);
  planner.set_ignore_other_robots(true);
  planner.set_debug_popup_enabled(false);

  auto queue_priority = [&](double g_cost, const Pose &pose)
  {
    if (!use_histogram || hist.empty())
      return g_cost;

    constexpr double kHistogramWeight = 3.0;
    double dx = pose.x - start_pose.x;
    double dy = pose.y - start_pose.y;
    double angle = (std::hypot(dx, dy) > 1e-6) ? std::atan2(dy, dx) : pose.yaw;
    return g_cost + kHistogramWeight * hist.sample_normalized(angle);
  };

  using PQElem = std::tuple<double, std::uint64_t, Node *, int>;
  auto cmp = [](const PQElem &a, const PQElem &b)
  {
    return std::get<0>(a) > std::get<0>(b) ||
           (std::get<0>(a) == std::get<0>(b) &&
            std::get<1>(a) > std::get<1>(b));
  };

  std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
  std::uint64_t node_id = 0;

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::unordered_map<std::string, double> best_cost_by_pose;
  best_cost_by_pose.emplace(pose_key(start_pose), 0.0);
  open_set.emplace(queue_priority(0.0, start_pose), node_id++, planner.start.get(), 0);

  int iterations = 0;
  while (!open_set.empty() && iterations < kMaxExpandIterations)
  {
    auto [priority_cost, _, current, depth] = open_set.top();
    open_set.pop();
    ++iterations;

    if (depth > kMaxPrimitiveDepth)
      continue;

    Pose current_pose{current->x, current->y, current->yaw};
    std::string current_key = pose_key(current_pose);
    auto best_it = best_cost_by_pose.find(current_key);
    if (best_it != best_cost_by_pose.end() &&
        current->cost > best_it->second + 1e-9)
    {
      continue;
    }

    if (depth > 0)
    {
      double displacement = std::hypot(current_pose.x - start_pose.x,
                                       current_pose.y - start_pose.y);
      if (displacement >= kMinParkingDisplacement)
      {
        PlanningResult trial_res;
        trial_res.status = PlanningStatus::SUCCESS;
        trial_res.waypoints =
            planner.extract_path(
                current, std::vector<std::tuple<double, double, double>>{});
        trial_res.failure_detail = use_histogram
                                       ? "Histogram-guided connected parking candidate"
                                       : "Connected primitive parking candidate";

        if (trial_res.waypoints.empty())
        {
          trial_res.status = PlanningStatus::NO_PATH_FOUND;
          trial_res.failure_detail = "Connected search reached pose but extracted no path";
          append_debug_trial(current_pose, trial_res, &timetable);
        }
        else if (parking_pose_conflicts_with_blocked_hint(current_pose, blocker,
                                                          blocked_traj_hint))
        {
          trial_res.status = PlanningStatus::NO_PATH_FOUND;
          trial_res.failure_detail =
              "Candidate pose still intersects the blocked path hint";
          trial_res.colliding_entity = "blocked_path_hint";
          trial_res.failure_time = ready_time;
          append_debug_trial(current_pose, trial_res, &timetable);
        }
        else
        {
          Trajectory trial_traj;
          trial_traj.entity = blocker;
          trial_traj.start_time = ready_time;
          trial_traj.waypoints = trial_res.waypoints;
          for (auto &wp : trial_traj.waypoints)
            wp.time -= ready_time;

          TimeTable trial_timetable = timetable;
          double wait_added = 0.0;
          CollisionInfo last_collision;
          double last_check_time = ready_time;
          double safe_start = find_wait_only_start_time(
              trial_traj, ready_time, trial_timetable, params,
              &last_collision, &last_check_time);
          wait_added = std::max(0.0, safe_start - ready_time);

          if (safe_start < 0.0)
          {
            const double scheduled_start = std::max(ready_time, last_check_time);
            shift_waypoint_times(trial_res.waypoints, scheduled_start - ready_time);
            trial_res.status = PlanningStatus::NO_PATH_FOUND;
            std::ostringstream oss;
            oss << "Connected parking path found, but no wait-only collision-free start slot";
            if (!last_collision.reason.empty())
            {
              oss << " (last: " << last_collision.reason;
              if (!last_collision.entity_name.empty())
                oss << " with " << last_collision.entity_name;
              if (last_collision.time > 1e-6)
                oss << " at t=" << std::fixed << std::setprecision(2)
                    << last_collision.time;
              oss << ")";
            }
            trial_res.failure_detail = oss.str();
            trial_res.colliding_entity = last_collision.entity_name;
            trial_res.failure_time =
                (last_collision.time > 1e-6) ? last_collision.time : last_check_time;
            append_debug_trial(current_pose, trial_res, &trial_timetable);
          }
          else
          {
            shift_waypoint_times(trial_res.waypoints, safe_start - ready_time);
            trial_traj.start_time = safe_start;
            double arrival_time = safe_start +
                                  (trial_traj.waypoints.empty() ? 0.0
                                                                : trial_traj.waypoints.back().time);
            CollisionInfo park_conflict =
                find_stationary_pose_conflict_until_last_timestamp(
                    blocker, current_pose, arrival_time, trial_timetable, params);
            if (!park_conflict.is_valid)
            {
              trial_res.status = PlanningStatus::NO_PATH_FOUND;
              std::ostringstream oss;
              oss << "Parking pose becomes occupied before the timetable ends";
              if (!park_conflict.reason.empty())
              {
                oss << " (" << park_conflict.reason;
                if (!park_conflict.entity_name.empty())
                  oss << " with " << park_conflict.entity_name;
                if (park_conflict.time > 1e-6)
                  oss << " at t=" << std::fixed << std::setprecision(2)
                      << park_conflict.time;
                oss << ")";
              }
              trial_res.failure_detail = oss.str();
              trial_res.colliding_entity = park_conflict.entity_name;
              trial_res.failure_time = park_conflict.time;
              append_debug_trial(current_pose, trial_res, &trial_timetable);
            }
            else
            {
              trial_timetable.add_trajectory(trial_traj);

              CollisionInfo blocked_hint_collision;
              double blocked_hint_check_time = ready_time;
              if (!parking_candidate_clears_blocked_hint(
                      blocked_traj_hint, blocker, trial_timetable, params,
                      &blocked_hint_collision, &blocked_hint_check_time))
              {
                trial_res.status = PlanningStatus::NO_PATH_FOUND;
                std::ostringstream oss;
                oss << "Parking candidate is feasible, but it does not clear the blocked path";
                if (!blocked_hint_collision.reason.empty())
                {
                  oss << " (" << blocked_hint_collision.reason;
                  if (!blocked_hint_collision.entity_name.empty())
                    oss << " with " << blocked_hint_collision.entity_name;
                  if (blocked_hint_collision.time > 1e-6)
                    oss << " at t=" << std::fixed << std::setprecision(2)
                        << blocked_hint_collision.time;
                  oss << ")";
                }
                trial_res.failure_detail = oss.str();
                trial_res.colliding_entity = blocked_hint_collision.entity_name;
                trial_res.failure_time =
                    (blocked_hint_collision.time > 1e-6)
                        ? blocked_hint_collision.time
                        : blocked_hint_check_time;
                append_debug_trial(current_pose, trial_res, &trial_timetable);
                continue;
              }

              trial_res.failure_detail = "Selected safe parking candidate";
              append_debug_trial(current_pose, trial_res, &trial_timetable);

              result.found = true;
              result.parking_pose = current_pose;
              result.committed_timetable = std::move(trial_timetable);
              return result;
            }
          }
        }
      }
    }

    if (depth >= kMaxPrimitiveDepth)
      continue;

    for (const auto &prim : planner.motion_primitives)
    {
      if (prim.first == 0)
        continue;

      Node *new_node = planner.generate_node(current, prim);
      if (!new_node)
        continue;
      if (!planner.check_collision_along_path(current, new_node, prim))
        continue;

      Pose next_pose{new_node->x, new_node->y, new_node->yaw};
      std::string next_key = pose_key(next_pose);
      auto cost_it = best_cost_by_pose.find(next_key);
      if (cost_it != best_cost_by_pose.end() &&
          new_node->cost + 1e-9 >= cost_it->second)
      {
        continue;
      }

      best_cost_by_pose[next_key] = new_node->cost;
      open_set.emplace(queue_priority(new_node->cost, next_pose),
                       node_id++, new_node, depth + 1);
    }
  }

  return result;
}

bool relocate_blocking_robot(RobotMeta *blocker,
                             TimeTable &timetable,
                             const Params &params,
                             const std::unordered_map<std::string, EntityMeta *> &entities,
                             const RuntimeOptions &options,
                             const Trajectory *blocked_traj_hint = nullptr)
{
  auto &recent_failed_relocations = recent_failed_relocation_cache();

  double ready_time = timetable.get_entity_max_time(blocker);
  // Add small buffer to start time to ensure no conflict with previous finish
  ready_time += 0.1;

  Pose start_pose = timetable.get_pose(blocker, ready_time);
  blocker->initial_pose = start_pose;

  auto make_fail_key = [&](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 10.0));
    int yi = static_cast<int>(std::round(p.y * 10.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 10.0));
    return blocker->name + "@" + std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::string fail_key = make_fail_key(start_pose);
  auto fail_it = recent_failed_relocations.find(fail_key);
  if (fail_it != recent_failed_relocations.end() && (ready_time - fail_it->second) < 10.0)
  {
    return false;
  }

  std::cout << "  [Relocate] Attempting to move " << blocker->name
            << " from (" << start_pose.x << ", " << start_pose.y << ")" << std::endl;

  TimeTable debug_timetable = timetable;
  std::vector<SafeParkingDebugTrial> debug_trials;
  auto record_debug_trial = [&](ParkingCandidateMode mode, const Pose &candidate_pose,
                                const PlanningResult &trial_result,
                                const TimeTable *context_timetable = nullptr)
  {
    if (!DEBUG_VIS)
      return;

    SafeParkingDebugTrial trial;
    trial.trial_index = static_cast<int>(debug_trials.size()) + 1;
    trial.candidate_mode = parking_candidate_mode_name(mode);
    trial.candidate_pose = candidate_pose;
    trial.result = trial_result;
    if (blocked_traj_hint)
      trial.blocked_path_hint_waypoints = blocked_traj_hint->waypoints;
    trial.context_timetable = context_timetable ? *context_timetable : timetable;
    debug_trials.push_back(std::move(trial));
  };
  auto show_debug_trials = [&]()
  {
    if (!DEBUG_VIS || debug_trials.empty())
      return;

    visualize_safe_parking_debug(debug_timetable, blocker, ready_time,
                                 start_pose, debug_trials, params);
  };

  std::unordered_set<std::string> tried_pose_keys;
  std::vector<ParkingCandidateMode> candidate_modes;
  auto append_mode_once = [&](ParkingCandidateMode mode)
  {
    if (std::find(candidate_modes.begin(), candidate_modes.end(), mode) == candidate_modes.end())
    {
      candidate_modes.push_back(mode);
    }
  };
  append_mode_once(g_parking_candidate_mode);
  if (g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT ||
      g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
  {
    append_mode_once(ParkingCandidateMode::CONNECTED_VFH);
    append_mode_once(ParkingCandidateMode::CONNECTED);
    append_mode_once(ParkingCandidateMode::EXPAND);
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else if (g_parking_candidate_mode == ParkingCandidateMode::CONNECTED_VFH)
  {
    append_mode_once(ParkingCandidateMode::CONNECTED);
    append_mode_once(ParkingCandidateMode::EXPAND);
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else if (g_parking_candidate_mode == ParkingCandidateMode::CONNECTED)
  {
    append_mode_once(ParkingCandidateMode::EXPAND);
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else if (g_parking_candidate_mode == ParkingCandidateMode::EXPAND)
  {
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else
  {
    append_mode_once(ParkingCandidateMode::EXPAND);
  }

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 10.0));
    int yi = static_cast<int>(std::round(p.y * 10.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 10.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  struct ParkingPathRefinement
  {
    bool found = false;
    Pose parking_pose;
    Trajectory trajectory;
    TimeTable committed_timetable;
    PlanningResult result;
  };

  auto try_shorten_reverse_recent_path =
      [&](const PlanningResult &scheduled_res) -> ParkingPathRefinement
  {
    ParkingPathRefinement refinement;
    if (scheduled_res.waypoints.size() < 3)
      return refinement;

    std::unordered_set<std::string> tested_prefix_poses;
    for (std::size_t i = 1; i + 1 < scheduled_res.waypoints.size(); ++i)
    {
      const auto &candidate_wp = scheduled_res.waypoints[i];
      Pose prefix_pose{candidate_wp.x, candidate_wp.y, candidate_wp.yaw};
      if (std::hypot(prefix_pose.x - start_pose.x,
                     prefix_pose.y - start_pose.y) < 0.25)
      {
        continue;
      }

      std::string key = pose_key(prefix_pose);
      if (!tested_prefix_poses.insert(key).second)
        continue;

      if (parking_pose_conflicts_with_blocked_hint(prefix_pose, blocker,
                                                   blocked_traj_hint))
      {
        continue;
      }

      std::vector<Waypoint> prefix_waypoints(
          scheduled_res.waypoints.begin(),
          scheduled_res.waypoints.begin() + static_cast<std::ptrdiff_t>(i + 1));
      const double original_abs_start = prefix_waypoints.front().time;
      make_waypoint_times_relative(prefix_waypoints, original_abs_start);

      Trajectory prefix_traj;
      prefix_traj.entity = blocker;
      prefix_traj.start_time = ready_time;
      prefix_traj.waypoints = prefix_waypoints;
      prefix_traj.is_transfer = false;
      prefix_traj.kind = TrajectoryKind::TRANSIT;
      prefix_traj.transferred_object = nullptr;
      prefix_traj.approach_goal_entity = nullptr;

      CollisionInfo last_collision;
      double last_check_time = ready_time;
      double prefix_safe_start = find_wait_only_start_time(
          prefix_traj, ready_time, timetable, params,
          &last_collision, &last_check_time);
      if (prefix_safe_start < 0.0)
        continue;

      prefix_traj.start_time = prefix_safe_start;
      const double arrival_time =
          prefix_safe_start +
          (prefix_traj.waypoints.empty() ? 0.0
                                         : prefix_traj.waypoints.back().time);

      CollisionInfo park_conflict =
          find_stationary_pose_conflict_until_last_timestamp(
              blocker, prefix_pose, arrival_time, timetable, params);
      if (!park_conflict.is_valid)
        continue;

      TimeTable prefix_timetable = timetable;
      prefix_timetable.add_trajectory(prefix_traj);

      CollisionInfo blocked_hint_collision;
      double blocked_hint_check_time = ready_time;
      if (!parking_candidate_clears_blocked_hint(
              blocked_traj_hint, blocker, prefix_timetable, params,
              &blocked_hint_collision, &blocked_hint_check_time))
      {
        continue;
      }

      PlanningResult prefix_result = scheduled_res;
      prefix_result.waypoints = prefix_waypoints;
      shift_waypoint_times(prefix_result.waypoints, prefix_safe_start);
      prefix_result.failure_detail =
          "Selected shorter reverse-recent parking candidate";
      prefix_result.colliding_entity.clear();
      prefix_result.failure_time = 0.0;

      refinement.found = true;
      refinement.parking_pose = prefix_pose;
      refinement.trajectory = std::move(prefix_traj);
      refinement.committed_timetable = std::move(prefix_timetable);
      refinement.result = std::move(prefix_result);
      return refinement;
    }

    return refinement;
  };

  for (std::size_t mode_index = 0; mode_index < candidate_modes.size(); ++mode_index)
  {
    ParkingCandidateMode candidate_mode = candidate_modes[mode_index];
    if (mode_index > 0)
    {
      std::cout << "  [Relocate] Retrying with "
                << parking_candidate_mode_name(candidate_mode)
                << " parking candidates." << std::endl;
    }

    if (candidate_mode == ParkingCandidateMode::CONNECTED ||
        candidate_mode == ParkingCandidateMode::CONNECTED_VFH)
    {
      auto connected_result = search_safe_parking_connected_search(
          candidate_mode, blocker, start_pose, ready_time,
          timetable, params, entities, blocked_traj_hint,
          DEBUG_VIS ? &debug_trials : nullptr);
      if (connected_result.found)
      {
        show_debug_trials();
        timetable = std::move(connected_result.committed_timetable);
        recent_failed_relocations.erase(fail_key);
        std::cout << "  [Relocate] SUCCESS: Moved " << blocker->name
                  << " to (" << connected_result.parking_pose.x << ", "
                  << connected_result.parking_pose.y << ")" << std::endl;
        return true;
      }
      continue;
    }

    auto candidates =
        generate_parking_candidates_for_mode(start_pose, blocker, params,
                                             candidate_mode, &timetable,
                                             &entities, ready_time);

    for (const auto &cand : candidates)
    {
      if (!std::isfinite(cand.estimated_rs_length))
        continue;

      if (std::hypot(cand.pose.x - start_pose.x, cand.pose.y - start_pose.y) < 0.25)
        continue;

      std::string key = pose_key(cand.pose);
      if (tried_pose_keys.count(key))
        continue;
      tried_pose_keys.insert(key);

      if (parking_pose_conflicts_with_blocked_hint(cand.pose, blocker,
                                                   blocked_traj_hint))
      {
        PlanningResult skipped_res;
        skipped_res.status = PlanningStatus::NO_PATH_FOUND;
        skipped_res.failure_detail =
            "Candidate pose still intersects the blocked path hint";
        skipped_res.colliding_entity = "blocked_path_hint";
        skipped_res.failure_time = ready_time;
        record_debug_trial(candidate_mode, cand.pose, skipped_res);
        continue;
      }

      PlanningResult res;
      if (!cand.connected_waypoints.empty())
      {
        res.status = PlanningStatus::SUCCESS;
        res.waypoints = cand.connected_waypoints;
        res.failure_detail = "Connected primitive parking candidate";
      }
      else
      {
        PHAStar planner(blocker, cand.pose, &timetable, &entities, params, false,
                        "", ready_time, "Safe parking relocation");
        planner.set_ignore_other_robots(true);
        planner.set_debug_popup_enabled(false);
        planner.max_search_iterations = options.max_search_iterations;
        planner.set_planner_expansion_threads(options.planner_expansion_threads);

        res = planner.Planning_with_res(ready_time);
      }

      if (res.status != PlanningStatus::SUCCESS)
      {
        record_debug_trial(candidate_mode, cand.pose, res);
        continue;
      }

      Trajectory relo_traj;
      relo_traj.entity = blocker;
      relo_traj.start_time = ready_time;
      relo_traj.waypoints = res.waypoints;
      relo_traj.is_transfer = false;
      relo_traj.kind = TrajectoryKind::TRANSIT;
      relo_traj.transferred_object = nullptr;
      relo_traj.approach_goal_entity = nullptr;
      for (auto &wp : relo_traj.waypoints)
        wp.time -= ready_time;

      double wait_added = 0.0;
      CollisionInfo last_collision;
      double last_check_time = ready_time;
      double safe_start = find_wait_only_start_time(
          relo_traj, ready_time, timetable, params,
          &last_collision, &last_check_time);
      wait_added = std::max(0.0, safe_start - ready_time);
      if (safe_start < 0.0)
      {
        PlanningResult trial_res = res;
        shift_waypoint_times(trial_res.waypoints,
                             std::max(ready_time, last_check_time) - ready_time);
        trial_res.status = PlanningStatus::NO_PATH_FOUND;
        std::ostringstream oss;
        oss << "Relocation path found, but no wait-only collision-free start slot";
        if (!last_collision.reason.empty())
        {
          oss << " (last: " << last_collision.reason;
          if (!last_collision.entity_name.empty())
            oss << " with " << last_collision.entity_name;
          if (last_collision.time > 1e-6)
            oss << " at t=" << std::fixed << std::setprecision(2)
                << last_collision.time;
          oss << ")";
        }
        trial_res.failure_detail = oss.str();
        trial_res.colliding_entity = last_collision.entity_name;
        trial_res.failure_time =
            (last_collision.time > 1e-6) ? last_collision.time : last_check_time;
        record_debug_trial(candidate_mode, cand.pose, trial_res);
        continue;
      }

      shift_waypoint_times(res.waypoints, safe_start - ready_time);
      relo_traj.start_time = safe_start;
      double arrival_time = safe_start + (relo_traj.waypoints.empty() ? 0.0 : relo_traj.waypoints.back().time);
      CollisionInfo park_conflict =
          find_stationary_pose_conflict_until_last_timestamp(
              blocker, cand.pose, arrival_time, timetable, params);
      if (!park_conflict.is_valid)
      {
        PlanningResult trial_res = res;
        trial_res.status = PlanningStatus::NO_PATH_FOUND;
        std::ostringstream oss;
        oss << "Parking pose becomes occupied before the timetable ends";
        if (!park_conflict.reason.empty())
        {
          oss << " (" << park_conflict.reason;
          if (!park_conflict.entity_name.empty())
            oss << " with " << park_conflict.entity_name;
          if (park_conflict.time > 1e-6)
            oss << " at t=" << std::fixed << std::setprecision(2)
                << park_conflict.time;
          oss << ")";
        }
        trial_res.failure_detail = oss.str();
        trial_res.colliding_entity = park_conflict.entity_name;
        trial_res.failure_time = park_conflict.time;
        record_debug_trial(candidate_mode, cand.pose, trial_res);
        continue;
      }

      PlanningResult success_res = res;
      Pose selected_parking_pose = cand.pose;
      TimeTable committed_timetable = timetable;
      committed_timetable.add_trajectory(relo_traj);

      CollisionInfo blocked_hint_collision;
      double blocked_hint_check_time = ready_time;
      if (!parking_candidate_clears_blocked_hint(
              blocked_traj_hint, blocker, committed_timetable, params,
              &blocked_hint_collision, &blocked_hint_check_time))
      {
        PlanningResult trial_res = res;
        trial_res.status = PlanningStatus::NO_PATH_FOUND;
        std::ostringstream oss;
        oss << "Parking candidate is feasible, but it does not clear the blocked path";
        if (!blocked_hint_collision.reason.empty())
        {
          oss << " (" << blocked_hint_collision.reason;
          if (!blocked_hint_collision.entity_name.empty())
            oss << " with " << blocked_hint_collision.entity_name;
          if (blocked_hint_collision.time > 1e-6)
            oss << " at t=" << std::fixed << std::setprecision(2)
                << blocked_hint_collision.time;
          oss << ")";
        }
        trial_res.failure_detail = oss.str();
        trial_res.colliding_entity = blocked_hint_collision.entity_name;
        trial_res.failure_time =
            (blocked_hint_collision.time > 1e-6)
                ? blocked_hint_collision.time
                : blocked_hint_check_time;
        record_debug_trial(candidate_mode, cand.pose, trial_res,
                           &committed_timetable);
        continue;
      }

      if (candidate_mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
      {
        ParkingPathRefinement refinement =
            try_shorten_reverse_recent_path(res);
        if (refinement.found)
        {
          selected_parking_pose = refinement.parking_pose;
          success_res = std::move(refinement.result);
          committed_timetable = std::move(refinement.committed_timetable);
          std::cout << "  [Relocate] Shortened reverse-recent parking from ("
                    << std::fixed << std::setprecision(2)
                    << cand.pose.x << ", " << cand.pose.y
                    << ") to (" << selected_parking_pose.x << ", "
                    << selected_parking_pose.y << ")." << std::endl;
        }
      }

      if (success_res.failure_detail.empty())
      {
        success_res.failure_detail = "Selected safe parking candidate";
      }
      record_debug_trial(candidate_mode, selected_parking_pose, success_res,
                         &committed_timetable);
      show_debug_trials();

      timetable = std::move(committed_timetable);
      recent_failed_relocations.erase(fail_key);
      std::cout << "  [Relocate] SUCCESS: Moved " << blocker->name
                << " to (" << selected_parking_pose.x << ", "
                << selected_parking_pose.y << ")" << std::endl;
      return true;
    }
  }
  recent_failed_relocations[fail_key] = ready_time;
  show_debug_trials();
  std::cerr << "  [Relocate] FAILED: Could not find safe parking spot for " << blocker->name << std::endl;
  return false;
}

// ==========================================
// Initial transit planning
// ==========================================
bool plan_initial_transit(
    RobotMeta *robot, const Pose &target_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    double *out_abs_start_time = nullptr,
    double *out_abs_end_time = nullptr,
    const std::function<Pose(double)> &target_pose_provider = {},
    const std::vector<Waypoint> *reference_waypoints = nullptr)
{
  if (out_abs_start_time)
    *out_abs_start_time = -1.0;
  if (out_abs_end_time)
    *out_abs_end_time = -1.0;

  double planning_start_time = start_time;
  double chosen_start_time = planning_start_time;
  Pose active_target_pose = target_pose;
  Pose current_pose = timetable.get_pose(robot, planning_start_time);
  robot->initial_pose = current_pose; // Update meta for planner

  std::cout << "  [Transit] Planning " << robot->name << " -> Target ("
            << active_target_pose.x << ", " << active_target_pose.y << ", " << active_target_pose.yaw
            << ") from Start (" << current_pose.x << ", " << current_pose.y << ", " << current_pose.yaw
            << ") at " << planning_start_time << "s" << std::endl;

  auto status_to_string = [](PlanningStatus status) -> const char *
  {
    switch (status)
    {
    case PlanningStatus::SUCCESS:
      return "SUCCESS";
    case PlanningStatus::START_INVALID_COLLISION:
      return "START_INVALID_COLLISION";
    case PlanningStatus::START_OUT_OF_BOUNDS:
      return "START_OUT_OF_BOUNDS";
    case PlanningStatus::GOAL_INVALID_COLLISION:
      return "GOAL_INVALID_COLLISION";
    case PlanningStatus::GOAL_OUT_OF_BOUNDS:
      return "GOAL_OUT_OF_BOUNDS";
    case PlanningStatus::NO_PATH_FOUND:
      return "NO_PATH_FOUND";
    case PlanningStatus::TIMEOUT_EXCEEDED:
      return "TIMEOUT_EXCEEDED";
    case PlanningStatus::HIGH_COST_UNFEASIBLE:
      return "HIGH_COST_UNFEASIBLE";
    case PlanningStatus::BLOCKED_BY_ROBOT:
      return "BLOCKED_BY_ROBOT";
    case PlanningStatus::INTERNAL_ERROR:
    default:
      return "INTERNAL_ERROR";
    }
  };

  std::vector<std::string> attempt_log;
  std::vector<PlanningDebugAttempt> debug_attempts;
  auto append_attempt = [&](const std::string &stage, const PlanningResult &res)
  {
    std::ostringstream oss;
    oss << stage << " => " << status_to_string(res.status);
    if (!res.colliding_entity.empty())
      oss << ", blocker=" << res.colliding_entity;
    if (res.failure_time > 1e-6)
      oss << ", t=" << std::fixed << std::setprecision(2) << res.failure_time;
    if (!res.failure_detail.empty())
      oss << ", detail=" << res.failure_detail;
    if (!res.waypoints.empty())
      oss << ", waypoints=" << res.waypoints.size();
    attempt_log.push_back(oss.str());
    debug_attempts.push_back({stage, res});
  };

  auto build_attempt_trace = [&]() -> std::string
  {
    if (attempt_log.empty())
      return "";

    std::ostringstream oss;
    oss << "Planning attempts:";
    for (std::size_t idx = 0; idx < attempt_log.size(); ++idx)
    {
      oss << "\n"
          << (idx + 1) << ") " << attempt_log[idx];
    }
    return oss.str();
  };

  auto refresh_target_pose = [&](double query_time,
                                 const std::string &reason_label) -> bool
  {
    if (!target_pose_provider)
      return false;

    const Pose refreshed = target_pose_provider(query_time);
    const double position_delta =
        std::hypot(refreshed.x - active_target_pose.x,
                   refreshed.y - active_target_pose.y);
    const double yaw_delta =
        std::abs(pi_2_pi(refreshed.yaw - active_target_pose.yaw));
    if (position_delta < 1e-3 && yaw_delta < 1e-3)
      return false;

    std::cout << "  [Transit] Refreshing initial transit target after "
              << reason_label << ": ("
              << std::fixed << std::setprecision(2)
              << active_target_pose.x << ", " << active_target_pose.y
              << ", " << active_target_pose.yaw << ") -> ("
              << refreshed.x << ", " << refreshed.y << ", "
              << refreshed.yaw << ") at t=" << query_time << "s."
              << std::endl;
    active_target_pose = refreshed;
    return true;
  };

  auto run_initial_transit_planner = [&](const std::string &debug_label,
                                         const Params &plan_params,
                                         int max_iterations) -> PlanningResult
  {
    current_pose = timetable.get_pose(robot, planning_start_time);
    robot->initial_pose = current_pose;
    PHAStar retry_planner(robot, active_target_pose, &timetable, &entities,
                          plan_params, false, "", planning_start_time, debug_label);
    retry_planner.set_debug_popup_enabled(false);
    retry_planner.max_search_iterations = max_iterations;
    retry_planner.set_planner_expansion_threads(options.planner_expansion_threads);
    if (options.enable_reference_egraph_transit && reference_waypoints &&
        reference_waypoints->size() >= 2)
    {
      retry_planner.set_reference_experience_graph(
          *reference_waypoints, reference_egraph_options_from_runtime(options));
      if (retry_planner.has_reference_experience_graph())
      {
        std::cout << "  [Transit] Reference E-Graph guide active for "
                  << debug_label << " (input waypoints="
                  << reference_waypoints->size() << ")." << std::endl;
      }
    }
    return retry_planner.Planning_with_res(planning_start_time);
  };

  auto replan_initial_transit = [&](const std::string &debug_label) -> PlanningResult
  {
    return run_initial_transit_planner(debug_label, params,
                                       options.max_search_iterations);
  };

  auto replan_initial_transit_fine = [&](const std::string &debug_label) -> PlanningResult
  {
    Params fine_params = make_fine_segment_params(params, options);
    const int fine_max_iter = std::max(
        options.max_search_iterations,
        options.fine_segment_max_search_iterations);
    return run_initial_transit_planner(debug_label, fine_params, fine_max_iter);
  };

  std::function<bool(PlanningResult &, const std::string &, bool)>
      try_schedule_time_aware_path =
          [&](PlanningResult &candidate_res,
              const std::string &stage_label,
              bool allow_target_refresh) -> bool
  {
    if (candidate_res.waypoints.empty())
    {
      return false;
    }

    Trajectory candidate_traj;
    candidate_traj.entity = robot;
    candidate_traj.is_transfer = false;
    candidate_traj.waypoints = candidate_res.waypoints;

    const double candidate_start_time = candidate_traj.waypoints.front().time;
    candidate_traj.start_time = candidate_start_time;
    make_waypoint_times_relative(candidate_traj.waypoints, candidate_start_time);

    double wait_added = 0.0;
    CollisionInfo last_collision;
    double last_check_time = candidate_start_time;
    double safe_start = find_safe_start_time(&candidate_traj, candidate_start_time,
                                             timetable, params, entities,
                                             options,
                                             &wait_added, &last_collision,
                                             &last_check_time,
                                             IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt);
    if (safe_start < 0.0)
    {
      PlanningResult sched_fail;
      sched_fail.status = PlanningStatus::NO_PATH_FOUND;
      sched_fail.colliding_entity = last_collision.entity_name;
      sched_fail.failure_time = last_check_time;
      std::ostringstream oss;
      oss << "time-aware scheduling failed";
      if (!last_collision.reason.empty())
      {
        oss << " (" << last_collision.reason;
        if (!last_collision.entity_name.empty())
        {
          oss << " with " << last_collision.entity_name;
        }
        oss << ")";
      }
      sched_fail.failure_detail = oss.str();
      append_attempt(stage_label, sched_fail);
      return false;
    }

    const double delta = safe_start - candidate_start_time;

    const double candidate_duration =
        candidate_res.waypoints.back().time - candidate_start_time;
    const double refreshed_target_query_time =
        safe_start + std::max(0.0, candidate_duration);
    if (allow_target_refresh && delta > 1e-6 &&
        refresh_target_pose(refreshed_target_query_time,
                            "delay recovery predicted arrival"))
    {
      planning_start_time = safe_start;
      chosen_start_time = safe_start;

      PlanningResult refreshed_res =
          replan_initial_transit("Initial transit after delay target refresh");
      append_attempt("replan after delayed target refresh", refreshed_res);

      if (!refreshed_res.waypoints.empty() &&
          try_schedule_time_aware_path(
              refreshed_res,
              "time-aware schedule of refreshed delayed path",
              false))
      {
        candidate_res = refreshed_res;
        return true;
      }

      candidate_res = refreshed_res;
      return false;
    }

    shift_waypoint_times(candidate_res.waypoints, delta);
    chosen_start_time = safe_start;
    candidate_res.status = PlanningStatus::SUCCESS;
    candidate_res.failure_detail.clear();
    candidate_res.colliding_entity.clear();
    candidate_res.failure_time = 0.0;
    append_attempt(stage_label, candidate_res);

    if (wait_added > 1e-9)
    {
      std::cout << "  [Transit] Dynamic blocker handled by waiting; scheduling path at "
                << std::fixed << std::setprecision(2) << safe_start
                << "s (wait " << wait_added << "s)." << std::endl;
    }
    else
    {
      std::cout << "  [Transit] Using time-aware scheduled path at "
                << std::fixed << std::setprecision(2) << safe_start
                << "s." << std::endl;
    }
    return true;
  };

  auto make_blocked_pose_hint = [&](const Pose &pose) -> Trajectory
  {
    Trajectory hint;
    hint.entity = robot;
    hint.start_time = planning_start_time;
    hint.is_transfer = false;

    Waypoint wp;
    wp.x = pose.x;
    wp.y = pose.y;
    wp.yaw = pose.yaw;
    wp.time = 0.0;
    wp.linear_velocity = 0.0;
    wp.steering_angle = 0.0;
    hint.waypoints.push_back(wp);
    return hint;
  };

  auto attempt_self_safe_parking = [&](PlanningResult &res,
                                       const Trajectory &blocked_hint,
                                       const std::string &reason_label,
                                       const std::string &retry_stage) -> bool
  {
    std::cout << "  [Transit] Preserving earlier reserved occupancy while "
              << robot->name << " adapts. " << reason_label
              << " Trying self safe parking before replanning initial transit."
              << std::endl;

    if (!relocate_blocking_robot(robot, timetable, params, entities,
                                 options, &blocked_hint))
    {
      std::cerr << "  [Transit] Self safe parking FAILED for " << robot->name
                << "." << std::endl;
      return false;
    }

    planning_start_time = timetable.get_entity_max_time(robot);
    chosen_start_time = planning_start_time;
    current_pose = timetable.get_pose(robot, planning_start_time);
    robot->initial_pose = current_pose;
    refresh_target_pose(planning_start_time, "self safe parking");

    std::cout << "  [Transit] " << robot->name
              << " parked safely. Replanning initial transit from ("
              << std::fixed << std::setprecision(2)
              << current_pose.x << ", " << current_pose.y << ", "
              << current_pose.yaw << ") at t=" << planning_start_time
              << "s." << std::endl;

    res = replan_initial_transit("Initial transit after self safe parking");
    append_attempt(retry_stage, res);
    return true;
  };

  auto attempt_validation_blocker_recovery = [&](PlanningResult &res,
                                                 const std::string &retry_stage) -> bool
  {
    if ((res.status != PlanningStatus::GOAL_INVALID_COLLISION &&
         res.status != PlanningStatus::START_INVALID_COLLISION) ||
        res.colliding_entity.empty())
    {
      return false;
    }

    const bool is_start_blocker =
        (res.status == PlanningStatus::START_INVALID_COLLISION);
    const Pose &blocked_pose = is_start_blocker ? current_pose : target_pose;
    const char *blocked_label = is_start_blocker ? "Start" : "Goal";
    const double blocker_time =
        (res.failure_time > 1e-6) ? res.failure_time : planning_start_time;

    CollisionInfo blocker_info;
    blocker_info.is_valid = false;
    blocker_info.reason =
        is_start_blocker ? "Start validation collision" : "Goal validation collision";
    blocker_info.entity_name = res.colliding_entity;
    blocker_info.time = blocker_time;

    RobotMeta *blocker = find_resolvable_robot_blocker(
        blocker_info, timetable, entities);
    if (!blocker)
    {
      return false;
    }

    const bool blocker_waiting = timetable.is_waiting(blocker, blocker_time);
    const bool blocker_static_after =
        timetable.is_entity_static_after(blocker_time, blocker);
    if (!blocker_waiting && !blocker_static_after)
    {
      std::cout << "  [Transit] " << blocked_label
                << " blocked by moving robot " << blocker->name
                << ". Relocation deferred; waiting-based recovery must handle it."
                << std::endl;
      return false;
    }

    Trajectory blocked_pose_hint = make_blocked_pose_hint(blocked_pose);
    std::cout << "  [Transit] " << blocked_label << " blocked by idle robot "
              << blocker->name;
    if (res.colliding_entity != blocker->name)
    {
      std::cout << " (reported as " << res.colliding_entity << ")";
    }
    std::cout << ". Keeping earlier reservation priority." << std::endl;
    return attempt_self_safe_parking(
        res, blocked_pose_hint,
        std::string(blocked_label) + " is occupied by higher-priority idle robot " +
            blocker->name + ".",
        retry_stage);
  };

  auto attempt_delayed_replan_after_blocked_backup =
      [&](PlanningResult &res,
          const std::string &reason_label) -> bool
  {
    const double original_start_time = planning_start_time;
    const double blocked_time =
        (res.failure_time > original_start_time + 1e-6)
            ? res.failure_time
            : original_start_time;

    std::cout << "  [Transit] " << reason_label
              << " Trying delayed replanning before self safe parking."
              << std::endl;

    constexpr int kMaxDelayedReplans = 2;
    constexpr double kDelayBuffer = 0.5;
    for (int delayed_attempt = 0; delayed_attempt < kMaxDelayedReplans;
         ++delayed_attempt)
    {
      planning_start_time =
          blocked_time + kDelayBuffer * static_cast<double>(delayed_attempt + 1);
      chosen_start_time = planning_start_time;
      refresh_target_pose(planning_start_time,
                          "blocked backup delayed replan");

      current_pose = timetable.get_pose(robot, planning_start_time);
      robot->initial_pose = current_pose;
      std::cout << "  [Transit] Delayed replan attempt "
                << (delayed_attempt + 1) << "/" << kMaxDelayedReplans
                << " for " << robot->name << " at t="
                << std::fixed << std::setprecision(2)
                << planning_start_time << "s from ("
                << current_pose.x << ", " << current_pose.y << ", "
                << current_pose.yaw << ")." << std::endl;

      PlanningResult delayed_res = replan_initial_transit(
          "Initial transit delayed replan after blocked backup");
      append_attempt("delayed replan after blocked backup", delayed_res);

      if (delayed_res.waypoints.empty())
      {
        if (options.enable_fine_segment_retry)
        {
          delayed_res = replan_initial_transit_fine(
              "Initial transit fine delayed replan after blocked backup");
          append_attempt("fine delayed replan after blocked backup",
                         delayed_res);
        }

        if (delayed_res.waypoints.empty())
        {
          continue;
        }
      }

      if (try_schedule_time_aware_path(
              delayed_res,
              "time-aware schedule of delayed blocked-backup replan",
              true))
      {
        res = delayed_res;
        std::cout << "  [Transit] Delayed replanning resolved the blocked "
                     "initial transit while preserving earlier reservations."
                  << std::endl;
        return true;
      }
    }

    planning_start_time = original_start_time;
    chosen_start_time = original_start_time;
    current_pose = timetable.get_pose(robot, planning_start_time);
    robot->initial_pose = current_pose;
    return false;
  };

  auto path_res = replan_initial_transit("Initial transit");
  append_attempt("primary planner", path_res);
  bool tried_fine_initial_transit = false;

  if (path_res.waypoints.empty() && options.enable_fine_segment_retry)
  {
    std::cout << "  [Transit] Primary initial transit failed. "
                 "Trying fine Hybrid A* before expensive fallbacks."
              << std::endl;
    path_res = replan_initial_transit_fine("Initial transit fine Hybrid A*");
    tried_fine_initial_transit = true;
    append_attempt("fine Hybrid A*", path_res);
  }

  for (int relocation_attempt = 0; relocation_attempt < 3; ++relocation_attempt)
  {
    if (!attempt_validation_blocker_recovery(
            path_res,
            "retry after self safe parking"))
    {
      break;
    }
  }

  // If blocked by robot, we already have a "ghost" path in path_res.waypoints
  if (path_res.status == PlanningStatus::BLOCKED_BY_ROBOT)
  {
    std::cout << "  [Transit] Path blocked by robot " << path_res.colliding_entity
              << ". Evaluating delay-only recovery before any self safe parking..."
              << std::endl;

    if (try_schedule_time_aware_path(path_res,
                                     "time-aware schedule of blocked path",
                                     true))
    {
      // Waiting already resolved the moving blocker, or the scheduler handled an idle blocker.
    }
    else
    {
      std::cout << "  [Transit] Time-aware scheduling could not clear the blocked path. "
                   "Checking whether the later initial transit must adapt..."
                << std::endl;

      if (attempt_delayed_replan_after_blocked_backup(
              path_res,
              "The blocked backup path could not be scheduled by waiting alone."))
      {
        // Fresh planning at a later start found a schedulable path.
      }
      else
      {
        Trajectory ghost_traj_abs;
        ghost_traj_abs.waypoints = path_res.waypoints;
        ghost_traj_abs.entity = robot;

        if (ghost_traj_abs.waypoints.empty())
        {
          std::cerr << " [Error] Blocked path has no waypoints! Cannot relocate." << std::endl;
          path_res.status = PlanningStatus::NO_PATH_FOUND;
        }
        else
        {
          Trajectory ghost_traj = ghost_traj_abs;
          double initial_t = ghost_traj.waypoints.front().time;
          ghost_traj.start_time = initial_t;
          ghost_traj_abs.start_time = initial_t;
          for (auto &wp : ghost_traj.waypoints)
            wp.time -= initial_t;

          CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);

          if (!col_info.is_valid && !col_info.entity_name.empty())
          {
            EntityMeta *collider = nullptr;
            auto collider_it = entities.find(col_info.entity_name);
            if (collider_it != entities.end())
              collider = collider_it->second;

            bool should_self_park = false;
            std::string blocker_label = col_info.entity_name;

            if (auto *blocking_robot = find_resolvable_robot_blocker(
                    col_info, timetable, entities))
            {
              should_self_park =
                  timetable.is_waiting(blocking_robot, col_info.time) ||
                  timetable.is_entity_static_after(col_info.time, blocking_robot);
              blocker_label = blocking_robot->name;
            }
            else if (collider && collider->type == EntityType::OBJECT)
            {
              should_self_park = true;
            }

            if (should_self_park &&
                attempt_self_safe_parking(
                    path_res, ghost_traj_abs,
                    "Higher-priority reserved occupancy from " + blocker_label +
                        " remains on the initial-transit corridor after delay-only scheduling.",
                    "retry after self safe parking from blocked path"))
            {
              std::cout << "  [Transit] Self safe parking recovery successful. Retrying plan..."
                        << std::endl;
            }
            else
            {
              std::cerr << "  [Transit] Preserved earlier reservation, but no self safe parking recovery was available."
                        << std::endl;
              path_res.waypoints.clear();
              path_res.status = PlanningStatus::NO_PATH_FOUND;
              path_res.failure_detail =
                  "delay-only scheduling failed and self safe parking recovery failed";
              append_attempt("self safe parking recovery", path_res);
            }
          }
        }
      }
    }
  }

  if (path_res.waypoints.empty() && !options.enable_initial_transit_fallbacks)
  {
    PlanningResult disabled_res;
    disabled_res.status = PlanningStatus::NO_PATH_FOUND;
    disabled_res.failure_detail = "initial-transit fallbacks disabled";
    append_attempt("initial-transit fallbacks disabled", disabled_res);
  }

  // Fallback for cases where standard planning finds nothing (Search exhausted)
  if (path_res.waypoints.empty() && options.enable_initial_transit_fallbacks)
  {
    std::cerr << " [Transit] Standard planning failed. Attempting to resolve blocking robots with full Ghost Planning..." << std::endl;

    // 1. Attempt Ghost Planning (Ignore other robots)
    PHAStar ghost_planner(robot, target_pose, &timetable, &entities, params,
                          false, "", planning_start_time,
                          "Initial transit ghost planning");
    ghost_planner.set_ignore_other_robots(true);
    ghost_planner.set_debug_popup_enabled(false);
    ghost_planner.max_search_iterations = options.max_search_iterations;
    ghost_planner.set_planner_expansion_threads(options.planner_expansion_threads);
    auto ghost_res = ghost_planner.Planning_with_res(planning_start_time);
    append_attempt("ghost planner", ghost_res);

    if (ghost_res.status == PlanningStatus::SUCCESS)
    {
      if (try_schedule_time_aware_path(ghost_res,
                                       "time-aware schedule of ghost path",
                                       true))
      {
        path_res = ghost_res;
      }
      else
      {
        // 2. Identify blockers on the ghost path
        Trajectory ghost_traj;
        ghost_traj.waypoints = ghost_res.waypoints;
        ghost_traj.entity = robot; // FIX: Prevent segfault in collision check
        double initial_t = ghost_traj.waypoints.front().time;
        ghost_traj.start_time = initial_t;
        make_waypoint_times_relative(ghost_traj.waypoints, initial_t);

        CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);

        if (!col_info.is_valid && !col_info.entity_name.empty())
        {
          RobotMeta *blocker = find_resolvable_robot_blocker(
              col_info, timetable, entities);
          EntityMeta *collider = nullptr;
          auto collider_it = entities.find(col_info.entity_name);
          if (collider_it != entities.end())
            collider = collider_it->second;

          bool should_self_park = false;
          std::string blocker_label = col_info.entity_name;
          if (blocker)
          {
            should_self_park =
                timetable.is_waiting(blocker, col_info.time) ||
                timetable.is_entity_static_after(col_info.time, blocker);
            blocker_label = blocker->name;
          }
          else if (collider && collider->type == EntityType::OBJECT)
          {
            should_self_park = true;
          }

          shift_waypoint_times(ghost_traj.waypoints, initial_t);
          ghost_traj.start_time = initial_t;

          if (should_self_park &&
              attempt_self_safe_parking(
                  path_res, ghost_traj,
                  "Ghost path shows higher-priority reserved occupancy from " +
                      blocker_label + " that delay-only scheduling cannot clear.",
                  "retry after self safe parking from ghost path"))
          {
            std::cout << "  [Transit] Self safe parking recovery successful on ghost path."
                      << std::endl;
          }
          else
          {
            std::cerr << "  [Transit] Ghost path blocked by unknown entity/boundary (" << col_info.entity_name << "). Cannot relocate." << std::endl;
          }
        }
      }
    }
  }

  if (path_res.waypoints.empty() && options.enable_initial_transit_fallbacks)
  {
    Params relaxed = make_relaxed_fallback_params(params);

    PHAStar geometry_planner(robot, target_pose, &timetable, &entities, relaxed,
                             false, "", planning_start_time,
                             "Initial transit geometry fallback");
    geometry_planner.set_ignore_other_robots(true);
    geometry_planner.set_debug_popup_enabled(false);
    geometry_planner.max_search_iterations = options.max_search_iterations;
    geometry_planner.set_planner_expansion_threads(options.planner_expansion_threads);
    auto geometry_res = geometry_planner.Planning_with_res(planning_start_time);
    append_attempt("geometry-first fallback", geometry_res);

    if (!geometry_res.waypoints.empty())
    {
      Trajectory geom_traj;
      geom_traj.entity = robot;
      geom_traj.is_transfer = false;
      geom_traj.start_time = planning_start_time;
      geom_traj.waypoints = geometry_res.waypoints;
      make_waypoint_times_relative(geom_traj.waypoints, planning_start_time);

      double wait_added = 0.0;
      double safe_start = find_safe_start_time(&geom_traj, planning_start_time, timetable,
                                               params, entities, options, &wait_added,
                                               nullptr, nullptr,
                                               IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt);
      if (safe_start >= 0.0)
      {
        double delta = safe_start - planning_start_time;
        shift_waypoint_times(geometry_res.waypoints, delta);
        path_res = geometry_res;
        chosen_start_time = safe_start;
        std::cout << "  [Transit] Using geometry-first fallback transit with scheduled start at "
                  << std::fixed << std::setprecision(2) << safe_start << "s" << std::endl;
      }
    }
  }

  if (path_res.waypoints.empty() && options.enable_initial_transit_fallbacks)
  {
    double maxc = 1.0 / std::max(robot->transit_turning_radius(), 1e-6);
    auto [rs_x, rs_y, rs_yaw, rs_ctypes, rs_lengths, rs_steers, rs_dirs] =
        ReedShepp::reeds_shepp_path_planning(current_pose.x, current_pose.y, current_pose.yaw,
                                             target_pose.x, target_pose.y, target_pose.yaw,
                                             maxc, params.rs_step_size, robot->wheel_base);

    if (!rs_x.empty())
    {
      std::vector<Waypoint> rs_waypoints;
      rs_waypoints.reserve(rs_x.size());
      double t_abs = planning_start_time;
      for (size_t i = 0; i < rs_x.size(); ++i)
      {
        if (i > 0)
        {
          double d = std::hypot(rs_x[i] - rs_x[i - 1], rs_y[i] - rs_y[i - 1]);
          t_abs += d / std::max(robot->speed_transit, 1e-3);
        }
        Waypoint wp;
        wp.x = rs_x[i];
        wp.y = rs_y[i];
        wp.yaw = rs_yaw[i];
        wp.time = t_abs;
        wp.linear_velocity = rs_dirs.empty() ? robot->speed_transit : rs_dirs[i] * robot->speed_transit;
        wp.steering_angle = rs_steers.empty() ? 0.0 : rs_steers[i];
        rs_waypoints.push_back(wp);
      }

      Trajectory rs_traj;
      rs_traj.entity = robot;
      rs_traj.is_transfer = false;
      rs_traj.start_time = planning_start_time;
      rs_traj.waypoints = rs_waypoints;
      make_waypoint_times_relative(rs_traj.waypoints, planning_start_time);

      double wait_added = 0.0;
      CollisionInfo last_collision;
      double last_check_time = planning_start_time;
      double safe_start = find_safe_start_time(&rs_traj, planning_start_time, timetable,
                                               params, entities, options, &wait_added,
                                               &last_collision, &last_check_time,
                                               IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt);
      if (safe_start >= 0.0)
      {
        path_res.waypoints = rs_waypoints;
        double delta = safe_start - planning_start_time;
        shift_waypoint_times(path_res.waypoints, delta);
        chosen_start_time = safe_start;
        path_res.status = PlanningStatus::SUCCESS;
        path_res.failure_detail.clear();
        path_res.colliding_entity.clear();
        path_res.failure_time = 0.0;
        append_attempt("RS fallback", path_res);
        std::cout << "  [Transit] Using RS fallback transit with scheduled start at "
                  << std::fixed << std::setprecision(2) << safe_start << "s" << std::endl;
      }
      else
      {
        PlanningResult rs_fail;
        rs_fail.status = PlanningStatus::NO_PATH_FOUND;
        rs_fail.failure_detail = "RS fallback found path but no collision-free start slot";
        rs_fail.waypoints = rs_waypoints;
        rs_fail.failure_time = last_check_time;
        rs_fail.colliding_entity = last_collision.entity_name;
        append_attempt("RS fallback", rs_fail);
      }
    }
    else
    {
      PlanningResult rs_fail;
      rs_fail.status = PlanningStatus::NO_PATH_FOUND;
      rs_fail.failure_detail = "RS fallback failed to find geometric path";
      append_attempt("RS fallback", rs_fail);
    }
  }

  const std::string attempt_trace = build_attempt_trace();

  if (path_res.waypoints.empty())
  {
    if (!attempt_trace.empty())
    {
      if (path_res.failure_detail.empty())
      {
        path_res.failure_detail = attempt_trace;
      }
      else
      {
        path_res.failure_detail += "\n" + attempt_trace;
      }
    }

    std::cerr << " [Error] Transit planning failed for " << robot->name
              << " - Status: " << static_cast<int>(path_res.status)
              << ", Detail: " << path_res.failure_detail << std::endl;
    diagnose_planning_failure(robot, current_pose, target_pose, planning_start_time, timetable, entities, params);
    if (DEBUG_VIS)
    {
      if (!debug_attempts.empty())
      {
        visualize_planning_attempt_debug(
            timetable,
            robot,
            planning_start_time,
            current_pose,
            target_pose,
            debug_attempts,
            params,
            "Initial transit");
      }
      else
      {
        visualize_planning_debug(
            timetable,
            robot,
            path_res,
            planning_start_time,
            current_pose,
            target_pose,
            params,
            attempt_trace,
            "Initial transit");
      }
    }
    return false;
  }

  auto validate_initial_transit_candidate =
      [&](PlanningResult &candidate_res,
          const std::string &stage_label) -> bool
  {
    if (candidate_res.waypoints.empty())
      return false;

    const double candidate_start_time = candidate_res.waypoints.front().time;
    Trajectory candidate_traj;
    candidate_traj.entity = robot;
    candidate_traj.start_time = candidate_start_time;
    candidate_traj.waypoints = candidate_res.waypoints;
    candidate_traj.is_transfer = false;
    candidate_traj.kind = TrajectoryKind::TRANSIT;
    make_waypoint_times_relative(candidate_traj.waypoints,
                                 candidate_start_time);

    CollisionInfo validation_info =
        check_collision_trajectory_detailed(candidate_traj,
                                            candidate_start_time,
                                            timetable, params, false);
    if (validation_info.is_valid)
      return true;

    PlanningResult validation_fail;
    validation_fail.status = PlanningStatus::NO_PATH_FOUND;
    validation_fail.failure_detail =
        "initial transit replay validation failed (" +
        validation_info.reason;
    if (!validation_info.entity_name.empty())
      validation_fail.failure_detail += " with " + validation_info.entity_name;
    validation_fail.failure_detail += ")";
    validation_fail.colliding_entity = validation_info.entity_name;
    validation_fail.failure_time = validation_info.time;
    validation_fail.waypoints = candidate_res.waypoints;
    append_attempt(stage_label, validation_fail);

    candidate_res.status = PlanningStatus::NO_PATH_FOUND;
    candidate_res.failure_detail = validation_fail.failure_detail;
    candidate_res.colliding_entity = validation_fail.colliding_entity;
    candidate_res.failure_time = validation_fail.failure_time;
    candidate_res.waypoints.clear();
    return false;
  };

  if (!validate_initial_transit_candidate(path_res,
                                          "initial transit replay validation"))
  {
    if (options.enable_fine_segment_retry && !tried_fine_initial_transit)
    {
      std::cout << "  [Transit] Initial transit replay validation failed. "
                   "Trying fine Hybrid A* repair before committing."
                << std::endl;
      path_res = replan_initial_transit_fine(
          "Initial transit fine Hybrid A* after replay validation");
      tried_fine_initial_transit = true;
      append_attempt("fine Hybrid A* after replay validation", path_res);
      validate_initial_transit_candidate(
          path_res,
          "fine initial transit replay validation");
    }
  }

  if (path_res.waypoints.empty())
  {
    const std::string updated_attempt_trace = build_attempt_trace();
    if (!updated_attempt_trace.empty())
    {
      if (path_res.failure_detail.empty())
        path_res.failure_detail = updated_attempt_trace;
      else
        path_res.failure_detail += "\n" + updated_attempt_trace;
    }

    std::cerr << " [Error] Transit planning failed for " << robot->name
              << " during replay validation - Status: "
              << static_cast<int>(path_res.status)
              << ", Detail: " << path_res.failure_detail << std::endl;
    if (DEBUG_VIS)
    {
      visualize_planning_attempt_debug(
          timetable,
          robot,
          planning_start_time,
          current_pose,
          target_pose,
          debug_attempts,
          params,
          "Initial transit");
    }
    return false;
  }

  if (DEBUG_VIS)
  {
    std::cout << "[Debug] Visualizing Plan..." << std::endl;

    if (!debug_attempts.empty())
    {
      visualize_planning_attempt_debug(
          timetable,
          robot,
          planning_start_time,
          current_pose,
          target_pose,
          debug_attempts,
          params,
          "Initial transit");
    }
    else
    {
      visualize_planning_debug(
          timetable,           // Global history/future of others
          robot,               // The robot executing this plan
          path_res,            // The output plan (waypoints)
          planning_start_time, // Absolute start time for this plan
          current_pose,        // Start
          target_pose,         // Goal
          params,
          attempt_trace,
          "Initial transit");
    }
  }

  // Adjust relative time and register
  make_waypoint_times_relative(path_res.waypoints, chosen_start_time);

  Trajectory transit_traj;
  transit_traj.entity = robot;
  transit_traj.start_time = chosen_start_time;
  transit_traj.waypoints = path_res.waypoints;
  transit_traj.is_transfer = false;
  timetable.add_trajectory(transit_traj);

  if (out_abs_start_time)
    *out_abs_start_time = chosen_start_time;
  if (out_abs_end_time)
  {
    *out_abs_end_time =
        path_res.waypoints.empty()
            ? chosen_start_time
            : (chosen_start_time + path_res.waypoints.back().time);
  }

  return true;
}

// ==========================================
// 2. CORE PLANNING SUB-ROUTINES
// ==========================================

// Finds the robot that becomes free the earliest
RobotMeta *find_earliest_robot(const std::vector<RobotMeta *> &robots,
                               TimeTable &timetable, double &out_free_time)
{
  RobotMeta *best_robot = nullptr;
  out_free_time = std::numeric_limits<double>::infinity();

  for (auto *robot : robots)
  {
    double t = timetable.get_entity_max_time(robot);
    if (t < out_free_time)
    {
      out_free_time = t;
      best_robot = robot;
    }
  }
  return best_robot;
}

// Returns list of (robot, free_time) sorted by earliest free time
std::vector<std::pair<RobotMeta *, double>> get_sorted_candidate_robots(
    const std::vector<RobotMeta *> &robots,
    TimeTable &timetable,
    const Task &task,
    const Params &params)
{
  std::vector<std::pair<RobotMeta *, double>> candidates;
  for (auto *robot : robots)
  {
    double t = timetable.get_entity_max_time(robot);
    candidates.emplace_back(robot, t);
  }

  auto rs_length_to_task = [&](RobotMeta *robot, double free_time)
  {
    Pose start_pose = timetable.get_pose(robot, free_time);
    Pose task_start_pose =
        compute_adjusted_task_start_pose(task, robot, free_time, timetable);
    double maxc = 1.0 / std::max(robot->transit_turning_radius(), 1e-6);
    double step_size = params.rs_step_size;
    auto [xs, ys, yaws, ctypes, lengths, steers, directions] =
        ReedShepp::reeds_shepp_path_planning(start_pose.x, start_pose.y, start_pose.yaw,
                                             task_start_pose.x, task_start_pose.y, task_start_pose.yaw,
                                             maxc, step_size, robot->wheel_base);
    if (xs.empty())
    {
      return INF;
    }
    double total = 0.0;
    for (double len : lengths)
    {
      total += std::abs(len);
    }
    return total;
  };

  std::sort(candidates.begin(), candidates.end(),
            [&](const auto &a, const auto &b)
            {
              constexpr double EPS = 1e-6;
              if (std::abs(a.second - b.second) > EPS)
              {
                return a.second < b.second;
              }
              double ra = rs_length_to_task(a.first, a.second);
              double rb = rs_length_to_task(b.first, b.second);
              if (std::abs(ra - rb) > EPS)
                return ra < rb;
              return a.first->name < b.first->name;
            });
  return candidates;
}

// Attempts to find a collision-free time slot for a trajectory segment
double find_safe_start_time(Trajectory *traj, double earliest_start,
                            TimeTable &timetable, const Params &params,
                            const std::unordered_map<std::string, EntityMeta *> &entities,
                            const RuntimeOptions &options,
                            double *out_wait_added,
                            CollisionInfo *out_last_collision,
                            double *out_last_check_time,
                            IdleBlockerRelocationPolicy idle_blocker_policy)
{
  if (out_wait_added)
    *out_wait_added = 0.0;
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  double check_time = earliest_start;
  double step = 0.5;
  constexpr double kExtraDelayBuffer = 0.5;

  std::string last_relocated_robot = "";
  double last_relocation_time = -100.0;
  CollisionInfo last_collision;
  double last_check_time = earliest_start;
  std::unordered_map<std::string, int> relocate_attempt_count;
  bool tried_boundary_projection = false;

  auto write_failure_outputs = [&]()
  {
    if (out_last_collision)
      *out_last_collision = last_collision;
    if (out_last_check_time)
      *out_last_check_time = last_check_time;
  };

  while (check_time <=
         timetable_delay_search_horizon(earliest_start, timetable, step) +
             1e-9)
  {
    CollisionInfo col_info = check_collision_trajectory_detailed(*traj, check_time, timetable, params, false);
    last_collision = col_info;
    last_check_time = check_time;

    if (col_info.is_valid)
    {
      double waited = std::max(0.0, check_time - earliest_start);

      if (waited > 1e-9)
      {
        const double buffered_start = check_time + kExtraDelayBuffer;
        CollisionInfo buffered_info =
            check_collision_trajectory_detailed(*traj, buffered_start,
                                                timetable, params, false);
        if (buffered_info.is_valid)
        {
          check_time = buffered_start;
          waited = std::max(0.0, check_time - earliest_start);
          col_info = buffered_info;
        }
      }

      if (out_wait_added)
        *out_wait_added = waited;
      if (out_last_collision)
        *out_last_collision = col_info;
      if (out_last_check_time)
        *out_last_check_time = check_time;
      if (waited > 1e-9)
        std::cout << "  [Delay] Delayed " << waited << "s for safety." << std::endl;
      return check_time;
    }

    // Handle Collision
    EntityMeta *collider = nullptr;
    if (entities.count(col_info.entity_name))
    {
      collider = entities.at(col_info.entity_name);
    }

    RobotMeta *resolvable_robot = find_resolvable_robot_blocker(
        col_info, timetable, entities);

    if (resolvable_robot)
    {
      RobotMeta *blocker = resolvable_robot;
      const bool blocker_is_waiting = timetable.is_waiting(blocker, col_info.time);
      const bool blocker_static_after =
          timetable.is_entity_static_after(col_info.time, blocker);
      if (blocker_is_waiting || blocker_static_after)
      {
        bool should_relocate_idle_blocker =
            idle_blocker_policy == IdleBlockerRelocationPolicy::RelocateAnyIdle;

        if (idle_blocker_policy ==
            IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt)
        {
          const bool blocker_waiting_at_attempt_start =
              timetable.is_waiting(blocker, earliest_start);
          const bool blocker_static_after_attempt_start =
              timetable.is_entity_static_after(earliest_start, blocker);
          const bool blocker_was_scheduled_to_move_at_attempt_start =
              !blocker_waiting_at_attempt_start &&
              !blocker_static_after_attempt_start;

          if (blocker_was_scheduled_to_move_at_attempt_start)
          {
            should_relocate_idle_blocker = true;
            std::cout << "  [Delay] " << blocker->name
                      << " was higher-priority traffic at candidate start t="
                      << std::fixed << std::setprecision(2) << earliest_start
                      << "s and became idle/static on "
                      << (traj && traj->entity ? traj->entity->name : "the trajectory")
                      << "'s corridor at t=" << col_info.time
                      << "s." << std::endl;
          }
        }

        if (!should_relocate_idle_blocker)
        {
          check_time += step;
          continue;
        }

        if (++relocate_attempt_count[blocker->name] > 3)
        {
          check_time += step;
          continue;
        }

        // Blocker is stationary/idle. Move it.
        if (blocker->name != last_relocated_robot || (check_time - last_relocation_time > 5.0))
        {
          if (idle_blocker_policy ==
              IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt)
          {
            std::cout << "  [Delay] Safe-parking became-idle blocker "
                      << blocker->name
                      << " before retrying "
                      << (traj && traj->entity ? traj->entity->name : "the trajectory")
                      << "'s initial-transit schedule." << std::endl;
          }

          if (relocate_blocking_robot(blocker, timetable, params, entities,
                                      options, traj))
          {
            last_relocated_robot = blocker->name;
            last_relocation_time = check_time;
            if (idle_blocker_policy ==
                IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt)
            {
              std::cout << "  [Delay] Retrying initial-transit scheduling after parking "
                        << blocker->name << "." << std::endl;
            }
            // Retry this time step (decrement so next loop increment checks same time)
            check_time -= step;
          }
        }
      }
      else
      {
        // Dynamic blocker is still moving. Keep waiting and re-evaluate.
      }
    }
    else if (collider && collider->type == EntityType::OBJECT)
    {
      bool object_will_move = !timetable.is_entity_static_after(col_info.time, collider);
      if (!object_will_move)
      {
        // Static object conflict cannot be solved by waiting.
        std::cerr << "  [Error] Static object " << collider->name
                  << " blocks path at t=" << std::fixed << std::setprecision(2)
                  << col_info.time << ". Waiting cannot resolve." << std::endl;
        write_failure_outputs();
        return -1.0;
      }
    }
    else if (col_info.entity_name == "Boundary" ||
             col_info.reason.find("Boundary") != std::string::npos)
    {
      if (!tried_boundary_projection)
      {
        tried_boundary_projection = true;
        RobotMeta *traj_robot = dynamic_cast<RobotMeta *>(traj->entity);
        if (traj_robot && !traj->waypoints.empty())
        {
          auto projected = traj->waypoints;
          project_waypoints_inside_bounds(projected, traj_robot, params);

          double max_shift = 0.0;
          for (size_t k = 0; k < projected.size(); ++k)
          {
            double dx = projected[k].x - traj->waypoints[k].x;
            double dy = projected[k].y - traj->waypoints[k].y;
            max_shift = std::max(max_shift, std::hypot(dx, dy));
          }

          if (max_shift > 1e-6)
          {
            traj->waypoints = std::move(projected);
            traj->CalcualteTimeStamps(traj_robot);
            std::cout << "  [Adjust] Projected segment path inside bounds and retrying." << std::endl;
            continue;
          }
        }
      }

      std::cerr << "  [Error] Trajectory collides with boundary at t="
                << std::fixed << std::setprecision(2) << col_info.time
                << ". Waiting cannot resolve." << std::endl;
      write_failure_outputs();
      return -1.0;
    }

    check_time += step;
  }

  std::cerr << "  [Error] Could not find safe slot through timetable horizon t="
            << std::fixed << std::setprecision(2)
            << timetable_delay_search_horizon(earliest_start, timetable, step)
            << "s (waited "
            << std::max(0.0, last_check_time - earliest_start)
            << "s). Last collision: "
            << last_collision.reason << " with " << last_collision.entity_name
            << " at t=" << last_collision.time << std::endl;
  write_failure_outputs();
  return -1.0; // Failure signal
}

// Generates and adds a retraction trajectory (backing up) after a push
bool append_retraction(RobotMeta *robot, const Trajectory &previous_traj,
                       TimeTable &timetable, const Params &params,
                       const std::unordered_map<std::string, EntityMeta *> &entities,
                       const RuntimeOptions &options,
                       TaskExecutionStats *stats = nullptr,
                       std::string *out_failure_reason = nullptr)
{
  if (!robot || previous_traj.waypoints.size() < 2)
    return true;

  const double retract_dist = 0.1; // Meters
  const double speed = std::max(robot->speed_transit, 1e-3);
  const size_t num_wp = previous_traj.waypoints.size();
  double actual_retract_dist = 0.0;

  // Backtrack along the just-executed push path. The previous implementation
  // often kept only the terminal waypoint, producing a 0s "retraction".
  std::vector<Waypoint> retract_wp;
  retract_wp.reserve(num_wp);
  Waypoint start_wp = previous_traj.waypoints.back();
  start_wp.time = 0.0;
  start_wp.linear_velocity = 0.0;
  retract_wp.push_back(start_wp);

  for (int i = static_cast<int>(num_wp) - 2;
       i >= 0 && actual_retract_dist < retract_dist - 1e-9;
       --i)
  {
    const Waypoint &toward_start = previous_traj.waypoints[i];
    const Waypoint &toward_end = previous_traj.waypoints[i + 1];
    const double dx = toward_start.x - toward_end.x;
    const double dy = toward_start.y - toward_end.y;
    const double segment_dist = std::hypot(dx, dy);
    if (segment_dist <= 1e-9)
      continue;

    const double remaining_dist = retract_dist - actual_retract_dist;
    const double frac = std::min(1.0, remaining_dist / segment_dist);

    Waypoint wp = toward_end;
    wp.x = toward_end.x + frac * dx;
    wp.y = toward_end.y + frac * dy;
    wp.yaw = mod2pi(toward_end.yaw +
                    frac * pi_2_pi(toward_start.yaw - toward_end.yaw));
    wp.linear_velocity = -speed;
    wp.steering_angle = -toward_end.steering_angle;
    retract_wp.push_back(wp);
    actual_retract_dist += frac * segment_dist;
  }

  if (retract_wp.size() < 2 || actual_retract_dist <= 1e-6)
    return true;

  // 3. Recalculate timing (assume transit speed)
  double current_time = 0.0;
  retract_wp[0].time = 0.0;
  for (size_t i = 1; i < retract_wp.size(); ++i)
  {
    double d = std::hypot(retract_wp[i].x - retract_wp[i - 1].x,
                          retract_wp[i].y - retract_wp[i - 1].y);
    current_time += (d / speed);
    retract_wp[i].time = current_time;
  }

  // 4. Add to timetable
  Trajectory retract_traj;
  retract_traj.entity = robot;
  double earliest_start = timetable.get_entity_max_time(robot);
  retract_traj.start_time = earliest_start;
  retract_traj.waypoints = retract_wp;
  retract_traj.is_transfer = false;
  retract_traj.kind = TrajectoryKind::RETRACTION;
  retract_traj.transferred_object = nullptr;
  retract_traj.approach_goal_entity = nullptr;

  double wait_added = 0.0;
  double safe_start = find_safe_start_time(&retract_traj, earliest_start,
                                           timetable, params, entities,
                                           options,
                                           &wait_added);
  if (safe_start < 0.0)
  {
    if (out_failure_reason)
      *out_failure_reason = "retraction has no collision-free slot";
    std::cerr << "  [Retract] Failed to schedule collision-free retraction." << std::endl;
    return false;
  }

  retract_traj.start_time = safe_start;
  if (stats && wait_added > 1e-9)
  {
    stats->total_waiting += wait_added;
    stats->delayed_segments += 1;
  }

  timetable.add_trajectory(retract_traj);
  std::ostringstream retract_msg;
  retract_msg << "  [Retract] Backing up " << std::fixed
              << std::setprecision(2) << actual_retract_dist << "m ("
              << current_time << "s).";
  std::cout << retract_msg.str() << std::endl;
  return true;
}

struct SegmentCandidateValidation
{
  bool hard_valid = false;
  bool has_soft_robot_conflict = false;
  CollisionInfo first_hard_collision;
  CollisionInfo first_soft_robot_collision;
};

struct SegmentCandidateReport
{
  std::string stage;
  PlanningResult planning;
  SegmentCandidateValidation validation;
  bool candidate_tested = false;
  bool accepted_for_scheduling = false;
  bool selected = false;
};

struct SegmentReplanContext
{
  int task_id = -1;
  int segment_id = -1;
  std::string object_name;
  std::string start_contact_entity;
  bool tight_or_contact_case = false;
};

bool is_soft_robot_collision(const CollisionInfo &info,
                             const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto it = entities.find(info.entity_name);
  if (it == entities.end() || !it->second)
    return false;
  return it->second->type == EntityType::ROBOT &&
         info.entity_name != "Boundary" &&
         info.reason.find("Boundary") == std::string::npos;
}

std::string find_valid_start_contact_entity(
    RobotMeta *robot,
    const Pose &start_pose,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto poses = timetable.get_poses(start_time);
  for (const auto &[name, entity] : entities)
  {
    if (!entity || entity->type != EntityType::OBJECT)
      continue;

    auto pose_it = poses.find(entity);
    if (pose_it == poses.end())
      continue;

    if (is_valid_transfer_contact(robot, start_pose, entity, pose_it->second))
      return name;
  }
  return "";
}

SegmentCandidateValidation validate_segment_candidate(
    const std::vector<Waypoint> &candidate_rel,
    RobotMeta *robot,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params)
{
  SegmentCandidateValidation validation;
  validation.first_hard_collision = {true, "Valid", "", start_time};
  validation.first_soft_robot_collision = {true, "Valid", "", start_time};

  if (candidate_rel.empty() || !robot)
  {
    validation.first_hard_collision =
        {false, "Empty or invalid candidate", "", start_time};
    return validation;
  }

  const double duration = std::max(0.0, candidate_rel.back().time);
  const double dt = shared_collision_check_step(params);

  auto validate_sample = [&](double rel_t) -> bool
  {
    const double abs_t = start_time + rel_t;
    auto pose_tuple = interpolate_timed_path(candidate_rel, rel_t);
    Pose robot_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                       std::get<2>(pose_tuple)};

    CollisionGeometry robot_geom = setup_collision_geometry_for_type(
        robot_pose, EntityType::ROBOT, robot->size, params);
    CollisionGeometry robot_bounds_geom =
        setup_collision_geometry(robot_pose, robot->size, 1.0);

    if (check_robot_bounds_collision(robot_pose, robot_bounds_geom.corners,
                                     params))
    {
      validation.first_hard_collision =
          {false, "Boundary Collision", "Boundary", abs_t};
      return false;
    }

    auto others = timetable.get_poses(abs_t);
    auto collision_result = check_multiple_entities_collision(
        robot_geom, robot_pose,
        nullptr, nullptr,
        others,
        params,
        robot,
        nullptr,
        nullptr);

    if (!collision_result.has_collision)
      return true;

    EntityMeta *collider = collision_result.colliding_entity;
    const std::string collider_name = collider ? collider->name : "";
    if (rel_t <= 0.3 && collider && collider->type == EntityType::OBJECT)
    {
      auto it = others.find(collider);
      if (it != others.end() &&
          is_valid_transfer_contact(robot, robot_pose, collider, it->second))
      {
        return true;
      }
    }

    CollisionInfo info{false, collision_result.collision_type + " Collision",
                       collider_name, abs_t};
    if (collider && collider->type == EntityType::ROBOT)
    {
      if (!validation.has_soft_robot_conflict)
      {
        validation.has_soft_robot_conflict = true;
        validation.first_soft_robot_collision = info;
      }
      return true;
    }

    validation.first_hard_collision = info;
    return false;
  };

  double rel_t = 0.0;
  while (rel_t < duration)
  {
    if (!validate_sample(rel_t))
      return validation;
    rel_t += dt;
  }
  if (!validate_sample(duration))
    return validation;

  validation.hard_valid = true;
  return validation;
}

std::vector<Waypoint> waypoints_from_rs_path(const ReedShepp::Path &path,
                                             RobotMeta *robot)
{
  std::vector<Waypoint> rel;
  rel.reserve(path.x.size());
  double t_rel = 0.0;
  for (std::size_t i = 0; i < path.x.size(); ++i)
  {
    if (i > 0)
    {
      const double d = std::hypot(path.x[i] - path.x[i - 1],
                                  path.y[i] - path.y[i - 1]);
      t_rel += d / std::max(robot->speed_transit, 1e-3);
    }

    Waypoint wp;
    wp.x = path.x[i];
    wp.y = path.y[i];
    wp.yaw = mod2pi(path.yaw[i]);
    wp.time = t_rel;
    const int direction = (i < path.directions.size()) ? path.directions[i] : 1;
    wp.linear_velocity = static_cast<double>(direction) * robot->speed_transit;
    wp.steering_angle = (i < path.steers.size()) ? path.steers[i] : 0.0;
    rel.push_back(wp);
  }
  return rel;
}

std::vector<Waypoint> make_reverse_escape_prefix(const Pose &start_pose,
                                                 RobotMeta *robot,
                                                 double steer,
                                                 double escape_distance)
{
  std::vector<Waypoint> prefix;
  if (!robot || escape_distance <= 1e-9)
    return prefix;

  constexpr int kEscapeSamples = 4;
  const double speed = std::max(robot->speed_transit, 1e-3);
  for (int i = 0; i <= kEscapeSamples; ++i)
  {
    const double frac = static_cast<double>(i) / kEscapeSamples;
    const double d = -escape_distance * frac;
    double x = start_pose.x;
    double y = start_pose.y;
    double yaw = start_pose.yaw;
    const double steer_adjusted = -steer;
    if (std::abs(steer_adjusted) < 1e-5)
    {
      x += d * std::cos(yaw);
      y += d * std::sin(yaw);
    }
    else
    {
      const double R = robot->wheel_base / std::tan(steer_adjusted);
      const double beta = d / R;
      x += R * (std::sin(yaw + beta) - std::sin(yaw));
      y += R * (std::cos(yaw) - std::cos(yaw + beta));
      yaw = mod2pi(yaw + beta);
    }

    Waypoint wp;
    wp.x = x;
    wp.y = y;
    wp.yaw = mod2pi(yaw);
    wp.time = escape_distance * frac / speed;
    wp.linear_velocity = (i == 0) ? 0.0 : -speed;
    wp.steering_angle = steer;
    prefix.push_back(wp);
  }
  return prefix;
}

std::vector<Waypoint> combine_prefix_and_tail(
    const std::vector<Waypoint> &prefix_rel,
    std::vector<Waypoint> tail_abs,
    double original_start_time)
{
  std::vector<Waypoint> combined = prefix_rel;
  make_waypoint_times_relative(tail_abs, original_start_time);
  for (std::size_t i = 0; i < tail_abs.size(); ++i)
  {
    if (!combined.empty() && i == 0)
      continue;
    combined.push_back(tail_abs[i]);
  }
  return combined;
}

std::string format_planner_stats(const PlanningDebugStats &stats)
{
  std::ostringstream oss;
  oss << "iterations=" << stats.iterations
      << ", generated=" << stats.generated_nodes
      << ", accepted=" << stats.accepted_nodes
      << ", closed=" << stats.closed_nodes
      << ", peak_open=" << stats.peak_open_size
      << ", reject_collision=" << stats.reject_collision
      << ", reject_closed=" << stats.reject_closed
      << ", reject_worse_g=" << stats.reject_worse_g
      << ", analytic_collision=" << stats.analytic_collision
      << ", analytic_post_arrival_collision="
      << stats.analytic_post_arrival_collision
      << ", egraph_nodes=" << stats.reference_egraph_nodes
      << ", egraph_snap_acc=" << stats.reference_egraph_snap_accepted
      << ", egraph_snap_rej=" << stats.reference_egraph_snap_rejected
      << ", egraph_succ_acc=" << stats.reference_egraph_successor_accepted
      << ", egraph_succ_rej=" << stats.reference_egraph_successor_rejected
      << ", mha_anchor_exp=" << stats.mha_anchor_expansions
      << ", mha_ref_exp=" << stats.mha_reference_expansions
      << ", mha_ref_queued=" << stats.mha_reference_queued
      << ", mha_ref_skip_far=" << stats.mha_reference_skipped_far
      << ", expansion_threads=" << stats.planner_expansion_threads
      << ", analytic_validation_time="
      << std::fixed << std::setprecision(4)
      << stats.analytic_validation_time_sec
      << "s, primitive_collision_time="
      << stats.primitive_collision_time_sec
      << "s, heuristic_time=" << stats.heuristic_time_sec
      << "s, serial_merge_time=" << stats.serial_merge_time_sec << "s";
  if (std::isfinite(stats.best_dist))
  {
    oss << ", best_dist=" << std::fixed << std::setprecision(3)
        << stats.best_dist
        << ", best_yaw_error=" << stats.best_yaw_error
        << ", best_pose=(" << stats.best_pose.x << ", "
        << stats.best_pose.y << ", " << stats.best_pose.yaw << ")";
  }
  if (!stats.final_failure_reason.empty())
    oss << ", final_failure=" << stats.final_failure_reason;
  return oss.str();
}

void write_segment_replan_diagnostics(
    const SegmentReplanContext &context,
    RobotMeta *robot,
    const Pose &start_pose,
    const Pose &goal_pose,
    double start_time,
    const std::vector<SegmentCandidateReport> &reports,
    bool success,
    const std::string &selected_stage)
{
  std::filesystem::create_directories(std::string(CMAKE_SOURCE_DIR) + "/results");
  std::ostringstream filename;
  filename << std::string(CMAKE_SOURCE_DIR) << "/results/planner_diagnostics_task"
           << context.task_id << "_" << sanitize_filename_component(context.object_name)
           << "_segment" << context.segment_id << "_"
           << sanitize_filename_component(robot ? robot->name : "unknown")
           << ".txt";

  std::ofstream ofs(filename.str());
  if (!ofs.is_open())
  {
    std::cerr << "[PlannerDiag] Failed to write " << filename.str() << std::endl;
    return;
  }

  ofs << "[PlannerDiag] Segment transit diagnostics\n";
  ofs << "[PlannerDiag] Result: " << (success ? "SUCCESS" : "FAILED") << "\n";
  ofs << "[PlannerDiag] Selected stage: " << selected_stage << "\n";
  ofs << "[PlannerDiag] Task: " << context.task_id
      << ", object: " << context.object_name
      << ", segment: " << context.segment_id << "\n";
  ofs << "[PlannerDiag] Robot: " << (robot ? robot->name : "unknown") << "\n";
  ofs << std::fixed << std::setprecision(3);
  if (robot)
  {
    ofs << "[PlannerDiag] Planner mode: transit, turning_radius="
        << robot->transit_turning_radius()
        << "m, transfer_turning_radius="
        << robot->transfer_turning_radius()
        << "m, legacy_radius=" << robot->min_turning_radius << "m\n";
  }
  ofs << "[PlannerDiag] Start: (" << start_pose.x << ", " << start_pose.y
      << ", " << start_pose.yaw << ") at t=" << start_time << "\n";
  ofs << "[PlannerDiag] Goal:  (" << goal_pose.x << ", " << goal_pose.y
      << ", " << goal_pose.yaw << ")\n";
  ofs << "[PlannerDiag] Start contact entity: "
      << (context.start_contact_entity.empty() ? "none"
                                               : context.start_contact_entity)
      << "\n";

  for (const auto &report : reports)
  {
    ofs << "[PlannerDiag] " << report.stage << ": "
        << planning_status_name(report.planning.status)
        << ", waypoints=" << report.planning.waypoints.size();
    if (!report.planning.failure_detail.empty())
      ofs << ", detail=" << report.planning.failure_detail;
    if (!report.planning.colliding_entity.empty())
      ofs << ", blocker=" << report.planning.colliding_entity;
    ofs << ", accepted_for_scheduling="
        << (report.accepted_for_scheduling ? "true" : "false")
        << ", selected=" << (report.selected ? "true" : "false")
        << "\n";
    ofs << "[PlannerDiag] " << report.stage
        << " stats: " << format_planner_stats(report.planning.debug_stats)
        << "\n";
    if (report.candidate_tested)
    {
      ofs << "[PlannerDiag] " << report.stage
          << " validation: hard_valid="
          << (report.validation.hard_valid ? "true" : "false")
          << ", soft_robot="
          << (report.validation.has_soft_robot_conflict ? "true" : "false");
      if (!report.validation.first_soft_robot_collision.is_valid)
      {
        ofs << ", first_soft="
            << report.validation.first_soft_robot_collision.reason << " with "
            << report.validation.first_soft_robot_collision.entity_name
            << " at t=" << report.validation.first_soft_robot_collision.time;
      }
      if (!report.validation.first_hard_collision.is_valid)
      {
        ofs << ", first_hard="
            << report.validation.first_hard_collision.reason << " with "
            << report.validation.first_hard_collision.entity_name
            << " at t=" << report.validation.first_hard_collision.time;
      }
      ofs << "\n";
    }
  }

  std::cout << "[PlannerDiag] Wrote segment diagnostics: "
            << filename.str() << std::endl;
}

bool replan_transit_segment(
    RobotMeta *robot, const Pose &goal_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<Waypoint> &out_waypoints_rel,
    const SegmentReplanContext &context = SegmentReplanContext{},
    const std::vector<Waypoint> *reference_waypoints = nullptr)
{
  Pose start_pose = timetable.get_pose(robot, start_time);
  robot->initial_pose = start_pose;
  SegmentReplanContext diag_context = context;
  diag_context.start_contact_entity = find_valid_start_contact_entity(
      robot, start_pose, start_time, timetable, entities);
  const double start_goal_dist =
      std::hypot(goal_pose.x - start_pose.x, goal_pose.y - start_pose.y);
  const double start_goal_yaw =
      std::fabs(pi_2_pi(goal_pose.yaw - start_pose.yaw));
  diag_context.tight_or_contact_case =
      !diag_context.start_contact_entity.empty() ||
      (start_goal_dist < 1.5 && start_goal_yaw > M_PI / 2.0);

  std::vector<SegmentCandidateReport> reports;
  std::string selected_stage;

  auto evaluate_candidate = [&](const std::string &stage,
                                PlanningResult planning,
                                std::vector<Waypoint> candidate_rel) -> bool
  {
    SegmentCandidateReport report;
    report.stage = stage;
    report.planning = std::move(planning);

    if (candidate_rel.empty())
    {
      reports.push_back(std::move(report));
      return false;
    }

    report.candidate_tested = true;
    report.validation = validate_segment_candidate(
        candidate_rel, robot, start_time, timetable, entities, params);
    report.accepted_for_scheduling = report.validation.hard_valid;
    if (!report.validation.hard_valid)
    {
      reports.push_back(std::move(report));
      return false;
    }

    out_waypoints_rel = std::move(candidate_rel);
    selected_stage = stage;
    report.selected = true;
    reports.push_back(std::move(report));
    return true;
  };

  auto run_hybrid_plan = [&](const Pose &plan_start_pose,
                             double plan_start_time,
                             const Params &plan_params,
                             int max_iter,
                             const std::string &stage) -> PlanningResult
  {
    Color::println("[PHAStar] Attempting search method: " + stage, Color::CYAN);
    robot->initial_pose = plan_start_pose;
    PHAStar planner(robot, goal_pose, &timetable, &entities, plan_params,
                    false, "", plan_start_time, stage);
    planner.set_ignore_other_robots(true);
    planner.max_search_iterations = max_iter;
    planner.set_planner_expansion_threads(options.planner_expansion_threads);
    if (options.enable_reference_egraph_transit && reference_waypoints &&
        reference_waypoints->size() >= 2)
    {
      planner.set_reference_experience_graph(
          *reference_waypoints, reference_egraph_options_from_runtime(options));
      if (planner.has_reference_experience_graph())
      {
        std::cout << "[PHAStar] Reference E-Graph guide active for "
                  << stage << " (input waypoints="
                  << reference_waypoints->size() << ")." << std::endl;
      }
    }
    return planner.Planning_with_res(plan_start_time);
  };

  auto evaluate_hybrid_stage = [&](const std::string &stage,
                                   const Pose &plan_start_pose,
                                   double plan_start_time,
                                   const Params &plan_params,
                                   int max_iter,
                                   const std::vector<Waypoint> *prefix = nullptr) -> bool
  {
    PlanningResult res = run_hybrid_plan(plan_start_pose, plan_start_time,
                                         plan_params, max_iter, stage);
    if (res.waypoints.empty())
      return evaluate_candidate(stage, res, {});

    std::vector<Waypoint> candidate_rel;
    if (prefix)
      candidate_rel = combine_prefix_and_tail(*prefix, res.waypoints, start_time);
    else
    {
      candidate_rel = res.waypoints;
      make_waypoint_times_relative(candidate_rel, start_time);
    }
    return evaluate_candidate(stage, res, std::move(candidate_rel));
  };

  if (evaluate_hybrid_stage("primary Hybrid A*", start_pose, start_time,
                            params, options.max_search_iterations))
  {
    if (diag_context.tight_or_contact_case)
      write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                       goal_pose, start_time, reports, true,
                                       selected_stage);
    return true;
  }

  if (options.enable_fine_segment_retry)
  {
    Params fine_params = make_fine_segment_params(params, options);
    const int fine_max_iter = std::max(
        options.max_search_iterations,
        options.fine_segment_max_search_iterations);
    if (diag_context.tight_or_contact_case &&
        evaluate_hybrid_stage("fine Hybrid A*", start_pose, start_time,
                              fine_params, fine_max_iter))
    {
      write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                       goal_pose, start_time, reports, true,
                                       selected_stage);
      return true;
    }

    if (!diag_context.start_contact_entity.empty())
    {
      const double escape_distance = 0.12;
      const double escape_steer =
          robot->transit_turning_radius() > 1e-9
              ? std::atan(robot->wheel_base / robot->transit_turning_radius())
              : 0.0;
      const std::vector<std::pair<std::string, double>> escape_stages = {
          {"reverse-straight escape + fine Hybrid A*", 0.0},
          {"reverse-left escape + fine Hybrid A*", escape_steer},
          {"reverse-right escape + fine Hybrid A*", -escape_steer}};

      for (const auto &[stage, steer_angle] : escape_stages)
      {
        auto prefix = make_reverse_escape_prefix(start_pose, robot,
                                                 steer_angle, escape_distance);
        if (prefix.empty())
          continue;
        const Waypoint &escape_end = prefix.back();
        Pose escape_pose{escape_end.x, escape_end.y, escape_end.yaw};
        const double escape_abs_time = start_time + escape_end.time;
        if (evaluate_hybrid_stage(stage, escape_pose, escape_abs_time,
                                  fine_params, fine_max_iter, &prefix))
        {
          write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                           goal_pose, start_time, reports, true,
                                           selected_stage);
          return true;
        }
      }
    }

    auto evaluate_rs_candidates = [&](const Pose &rs_start_pose,
                                      double rs_start_time,
                                      const Params &rs_params,
                                      const std::string &stage_prefix,
                                      const std::vector<Waypoint> *prefix = nullptr) -> bool
    {
      Color::println("[PHAStar] Attempting search method: " + stage_prefix, Color::CYAN);
      const double maxc =
          1.0 / std::max(robot->transit_turning_radius(), 1e-6);
      auto paths = ReedShepp::calc_paths(rs_start_pose.x, rs_start_pose.y,
                                         rs_start_pose.yaw,
                                         goal_pose.x, goal_pose.y, goal_pose.yaw,
                                         maxc, rs_params.rs_step_size,
                                         robot->wheel_base);
      std::sort(paths.begin(), paths.end(),
                [](const ReedShepp::Path &a, const ReedShepp::Path &b)
                { return a.L < b.L; });

      for (std::size_t i = 0; i < paths.size(); ++i)
      {
        PlanningResult rs_res;
        rs_res.status = PlanningStatus::SUCCESS;
        rs_res.waypoints = waypoints_from_rs_path(paths[i], robot);
        shift_waypoint_times(rs_res.waypoints, rs_start_time);

        std::vector<Waypoint> candidate_rel;
        if (prefix)
          candidate_rel = combine_prefix_and_tail(*prefix, rs_res.waypoints,
                                                  start_time);
        else
        {
          candidate_rel = rs_res.waypoints;
          make_waypoint_times_relative(candidate_rel, start_time);
        }

        std::ostringstream stage;
        stage << stage_prefix << " RS candidate " << i << " "
              << paths[i].ctypes << " L=" << std::fixed << std::setprecision(3)
              << paths[i].L;
        if (evaluate_candidate(stage.str(), rs_res, std::move(candidate_rel)))
          return true;
      }
      return false;
    };

    if (evaluate_rs_candidates(start_pose, start_time, fine_params,
                               "all-candidate"))
    {
      write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                       goal_pose, start_time, reports, true,
                                       selected_stage);
      return true;
    }

    if (!diag_context.start_contact_entity.empty())
    {
      const double escape_distance = 0.12;
      const double escape_steer =
          robot->transit_turning_radius() > 1e-9
              ? std::atan(robot->wheel_base / robot->transit_turning_radius())
              : 0.0;
      const std::vector<std::pair<std::string, double>> escape_stages = {
          {"reverse-straight escape", 0.0},
          {"reverse-left escape", escape_steer},
          {"reverse-right escape", -escape_steer}};
      for (const auto &[stage, steer_angle] : escape_stages)
      {
        auto prefix = make_reverse_escape_prefix(start_pose, robot,
                                                 steer_angle, escape_distance);
        if (prefix.empty())
          continue;
        const Waypoint &escape_end = prefix.back();
        Pose escape_pose{escape_end.x, escape_end.y, escape_end.yaw};
        const double escape_abs_time = start_time + escape_end.time;
        if (evaluate_rs_candidates(escape_pose, escape_abs_time, fine_params,
                                   stage, &prefix))
        {
          write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                           goal_pose, start_time, reports, true,
                                           selected_stage);
          return true;
        }
      }
    }
  }

  write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                   goal_pose, start_time, reports, false,
                                   selected_stage.empty() ? "none"
                                                          : selected_stage);
  return false;
}

// ==========================================
// 3. MAIN TASK PIPELINE
// ==========================================

bool prepare_segment_waypoints_for_scheduling(
    Trajectory *traj,
    RobotMeta *robot,
    double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    const std::string &fallback_message,
    const SegmentReplanContext &replan_context = SegmentReplanContext{})
{
  if (!traj)
    return false;

  traj->entity = robot;
  if (!traj->is_transfer)
  {
    if (traj->waypoints.empty())
      return false;

    auto original_waypoints = traj->waypoints;
    Pose start_pose = timetable.get_pose(robot, segment_ready_time);
    Pose segment_goal = traj->waypoints.back();
    EntityMeta *approach_goal_entity = traj->approach_goal_entity;
    if (robot && approach_goal_entity)
    {
      const Pose live_object_pose =
          timetable.get_pose(approach_goal_entity, segment_ready_time);
      segment_goal = compute_adjusted_prepush_goal(
          live_object_pose, segment_goal.yaw, robot, approach_goal_entity,
          0.01);
    }
    std::vector<Waypoint> reference_waypoints = original_waypoints;
    if (!reference_waypoints.empty())
    {
      reference_waypoints.front().x = start_pose.x;
      reference_waypoints.front().y = start_pose.y;
      reference_waypoints.front().yaw = start_pose.yaw;
      reference_waypoints.back().x = segment_goal.x;
      reference_waypoints.back().y = segment_goal.y;
      reference_waypoints.back().yaw = segment_goal.yaw;
    }
    std::vector<Waypoint> replanned_rel;
    if (!replan_transit_segment(robot, segment_goal, segment_ready_time,
                                timetable, entities, params, options, replanned_rel,
                                replan_context,
                                &reference_waypoints))
    {
      std::cout << fallback_message << std::endl;
      traj->waypoints = original_waypoints;
      return false;
    }
    else
    {
      traj->waypoints = replanned_rel;
      traj->transferred_object = nullptr;
    }
  }
  else
  {
    rewrite_transfer_terminal_pose(traj, robot);
    traj->CalcualteTimeStamps(robot);
  }

  return true;
}

bool replan_transfer_segment_after_failed_schedule(
    Trajectory *traj,
    RobotMeta *robot,
    double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::string *out_failure_reason = nullptr)
{
  if (!traj || !robot || !traj->is_transfer || !traj->transferred_object ||
      traj->waypoints.empty())
  {
    if (out_failure_reason)
      *out_failure_reason = "transfer replanning skipped: invalid segment";
    return false;
  }

  const Pose start_pose = timetable.get_pose(robot, segment_ready_time);
  const Pose goal_pose = traj->waypoints.back();
  robot->initial_pose = start_pose;

  std::cout << "  [Segment] Given transfer path is not schedulable; "
            << "attempting forward-only transfer replanning for "
            << traj->transferred_object->name << "." << std::endl;

  PHAStar planner(robot, goal_pose, &timetable, &entities, params,
                  true, traj->transferred_object->name, segment_ready_time,
                  "Transfer recovery");
  planner.max_search_iterations = options.max_search_iterations;
  planner.set_planner_expansion_threads(options.planner_expansion_threads);

  PlanningResult replanned = planner.Planning_with_res(segment_ready_time);
  if (replanned.status != PlanningStatus::SUCCESS ||
      replanned.waypoints.empty())
  {
    if (out_failure_reason)
    {
      std::ostringstream oss;
      oss << "transfer replanning failed: "
          << planning_status_name(replanned.status);
      if (!replanned.failure_detail.empty())
        oss << " (" << replanned.failure_detail << ")";
      *out_failure_reason = oss.str();
    }
    std::cerr << "  [Segment] Transfer replanning failed." << std::endl;
    return false;
  }

  make_waypoint_times_relative(replanned.waypoints, segment_ready_time);
  traj->waypoints = std::move(replanned.waypoints);
  traj->start_time = segment_ready_time;
  traj->kind = TrajectoryKind::TRANSFER;
  traj->is_transfer = true;

  std::cout << "  [Segment] Transfer replanning produced "
            << traj->waypoints.size()
            << " forward-only waypoints; retrying scheduling." << std::endl;
  return true;
}

bool reserve_and_commit_trajectory(
    Trajectory *traj,
    RobotMeta *robot,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows = nullptr,
    TaskExecutionStats *stats = nullptr,
    std::string *out_failure_reason = nullptr,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr)
{
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  traj->kind = traj->is_transfer ? TrajectoryKind::TRANSFER : traj->kind;

  if (traj->is_transfer && traj->transferred_object)
  {
    constexpr double kTransferReservationStep = 0.5;

    double candidate_earliest_start = earliest_start;
    std::string transfer_failure_reason =
        "failed to reserve parked transfer object against higher-priority traffic";

    while (candidate_earliest_start <=
           timetable_delay_search_horizon(earliest_start, timetable,
                                          kTransferReservationStep) +
               1e-9)
    {
      TimeTable trial_timetable = timetable;

      double wait_added = 0.0;
      CollisionInfo last_schedule_collision;
      double last_schedule_check_time = candidate_earliest_start;
      double safe_start_time =
          find_safe_start_time(traj, candidate_earliest_start,
                               trial_timetable, params, entities, options,
                               &wait_added, &last_schedule_collision,
                               &last_schedule_check_time);
      if (safe_start_time < 0.0)
      {
        if (out_last_collision)
          *out_last_collision = last_schedule_collision;
        if (out_last_check_time)
          *out_last_check_time = last_schedule_check_time;

        if (!last_schedule_collision.reason.empty() &&
            last_schedule_collision.reason != "Not evaluated")
        {
          std::ostringstream oss;
          oss << "failed to find safe start for transfer segment";
          if (!last_schedule_collision.reason.empty())
          {
            oss << " (" << last_schedule_collision.reason;
            if (!last_schedule_collision.entity_name.empty())
              oss << " with " << last_schedule_collision.entity_name;
            if (last_schedule_collision.time > 1e-6)
              oss << " at t=" << std::fixed << std::setprecision(2)
                  << last_schedule_collision.time;
            oss << ")";
          }
          transfer_failure_reason = oss.str();
        }
        break;
      }

      traj->start_time = safe_start_time;
      traj->CalcualteTimeStamps(robot);
      trial_timetable.add_trajectory(*traj);

      const double segment_duration =
          traj->waypoints.empty() ? 0.0 : traj->waypoints.back().time;
      const double arrival_time = safe_start_time + segment_duration;
      const Pose parked_pose =
          trial_timetable.get_pose(traj->transferred_object, arrival_time);

      bool retry_with_delayed_start = false;
      while (true)
      {
        CollisionInfo park_conflict =
            find_stationary_pose_conflict_until_last_timestamp_ignoring(
                traj->transferred_object, parked_pose, arrival_time,
                trial_timetable, params, shared_collision_check_step(params),
                robot);
        if (park_conflict.is_valid)
        {
          accumulate_wait_stats(stats,
                                std::max(0.0, safe_start_time - earliest_start));
          if (out_last_collision)
            *out_last_collision = park_conflict;
          if (out_last_check_time)
            *out_last_check_time = safe_start_time;
          timetable = std::move(trial_timetable);
          append_transfer_window_if_needed(*traj, transfer_windows);
          return true;
        }

        if (out_last_collision)
          *out_last_collision = park_conflict;
        if (out_last_check_time)
          *out_last_check_time = park_conflict.time;

        EntityMeta *collider = nullptr;
        auto collider_it = entities.find(park_conflict.entity_name);
        if (collider_it != entities.end())
          collider = collider_it->second;

        if (auto *blocking_robot = dynamic_cast<RobotMeta *>(collider))
        {
          const bool blocker_waiting =
              trial_timetable.is_waiting(blocking_robot, park_conflict.time);
          const bool blocker_static_after =
              trial_timetable.is_entity_static_after(park_conflict.time,
                                                     blocking_robot);
          if (blocker_waiting || blocker_static_after)
          {
            std::cout << "  [Segment] Parked "
                      << traj->transferred_object->name
                      << " conflicts with idle robot " << blocking_robot->name
                      << ". Relocating blocker first." << std::endl;
            if (relocate_blocking_robot(blocking_robot, trial_timetable, params,
                                        entities, options))
            {
              continue;
            }

            std::ostringstream oss;
            oss << "failed to relocate idle robot " << blocking_robot->name
                << " blocking parked " << traj->transferred_object->name;
            transfer_failure_reason = oss.str();
            break;
          }

          const double delayed_start =
              delayed_segment_start_after_stationary_conflict(
                  safe_start_time, segment_duration, park_conflict.time,
                  kTransferReservationStep);
          std::cout << "  [Segment] Delaying transfer start by "
                    << std::fixed << std::setprecision(2)
                    << (delayed_start - safe_start_time)
                    << "s so parked " << traj->transferred_object->name
                    << " clears higher-priority " << blocking_robot->name
                    << " at t=" << park_conflict.time << "s." << std::endl;
          candidate_earliest_start = delayed_start;
          retry_with_delayed_start = true;
          break;
        }

        if (collider && collider->type == EntityType::OBJECT)
        {
          const bool collider_static_after =
              trial_timetable.is_entity_static_after(park_conflict.time,
                                                     collider);
          if (!collider_static_after)
          {
            const double delayed_start =
                delayed_segment_start_after_stationary_conflict(
                    safe_start_time, segment_duration, park_conflict.time,
                    kTransferReservationStep);
            std::cout << "  [Segment] Delaying transfer start by "
                      << std::fixed << std::setprecision(2)
                      << (delayed_start - safe_start_time)
                      << "s so parked " << traj->transferred_object->name
                      << " clears higher-priority object " << collider->name
                      << " at t=" << park_conflict.time << "s." << std::endl;
            candidate_earliest_start = delayed_start;
            retry_with_delayed_start = true;
            break;
          }

          std::ostringstream oss;
          oss << "parked " << traj->transferred_object->name
              << " conflicts with reserved object " << collider->name;
          transfer_failure_reason = oss.str();
          break;
        }

        if (park_conflict.entity_name == "Boundary" ||
            park_conflict.reason.find("Boundary") != std::string::npos)
        {
          transfer_failure_reason =
              "parked transfer object would end up out of bounds";
        }
        else if (!park_conflict.entity_name.empty())
        {
          std::ostringstream oss;
          oss << "parked " << traj->transferred_object->name
              << " conflicts with reserved entity "
              << park_conflict.entity_name;
          transfer_failure_reason = oss.str();
        }
        break;
      }

      if (retry_with_delayed_start)
        continue;

      break;
    }

    if (out_failure_reason)
      *out_failure_reason = transfer_failure_reason;
    std::cerr << "  [Segment] Failed to reserve parked transfer object safely."
              << std::endl;
    return false;
  }

  double wait_added = 0.0;
  double safe_start_time =
      find_safe_start_time(traj, earliest_start, timetable, params, entities,
                           options, &wait_added, out_last_collision,
                           out_last_check_time);
  accumulate_wait_stats(stats, wait_added);

  if (safe_start_time < 0.0)
  {
    if (out_failure_reason)
      *out_failure_reason = "failed to find safe start for scheduled segment";
    std::cerr << "  [Segment] Failed to find safe start time for path segment."
              << std::endl;
    return false;
  }

  traj->start_time = safe_start_time;
  timetable.add_trajectory(*traj);
  append_transfer_window_if_needed(*traj, transfer_windows);
  return true;
}

// Helper function to handle the scheduling of a single path segment
bool schedule_path_segment(const EdgePath &edge_path, EntityMeta *obj_meta,
                           RobotMeta *robot, TimeTable &timetable,
                           const Params &params,
                           const std::unordered_map<std::string, EntityMeta *> &entities,
                           const RuntimeOptions &options,
                           double source_pre_push_distance = 0.0,
                           EntityMeta *approach_goal_entity = nullptr,
                           std::vector<TransferContactWindow> *transfer_windows = nullptr,
                           TaskExecutionStats *stats = nullptr,
                           std::string *out_failure_reason = nullptr,
                           double *out_scheduled_start_time = nullptr,
                           Trajectory *out_scheduled_trajectory = nullptr)
{
  // 1. Get current available time from the timetable
  double current_avail_time = timetable.get_entity_max_time(robot);

  // 2. Convert EdgePath to Trajectory
  // ReloPushPath2TrajPtr is defined in Task.h
  TrajectoryPtr traj =
      ReloPushPath2TrajPtr(edge_path, robot, obj_meta, current_avail_time,
                           source_pre_push_distance, approach_goal_entity);

  if (!prepare_segment_waypoints_for_scheduling(
          traj.get(), robot, current_avail_time, timetable, entities, params,
          options,
          "  [Segment] Transit replanning failed; using original transit path with conflict-resolution scheduling."))
  {
    if (out_failure_reason)
      *out_failure_reason = "empty transit segment";
    return false;
  }

  CollisionInfo segment_failure_collision;
  double segment_failure_start = current_avail_time;
  bool success = reserve_and_commit_trajectory(
      traj.get(), robot, current_avail_time, timetable, params, entities,
      options, transfer_windows, stats, out_failure_reason,
      &segment_failure_collision, &segment_failure_start);

  if (!success && traj->is_transfer && traj->transferred_object)
  {
    std::string transfer_replan_failure;
    if (replan_transfer_segment_after_failed_schedule(
            traj.get(), robot, current_avail_time, timetable, entities,
            params, options, &transfer_replan_failure))
    {
      success = reserve_and_commit_trajectory(
          traj.get(), robot, current_avail_time, timetable, params, entities,
          options, transfer_windows, stats, out_failure_reason,
          &segment_failure_collision, &segment_failure_start);
    }
    else if (out_failure_reason && !transfer_replan_failure.empty())
    {
      *out_failure_reason = transfer_replan_failure;
    }
  }

  if (success)
  {
    if (out_scheduled_start_time)
      *out_scheduled_start_time = traj->start_time;
    if (out_scheduled_trajectory)
      *out_scheduled_trajectory = *traj;
  }
  return success;
}

bool process_task_execution(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows = nullptr,
    TaskExecutionStats *out_stats = nullptr,
    std::string *out_failure_reason = nullptr,
    double task_start_delay = 0.0,
    int task_id = -1)
{
  if (out_stats)
    *out_stats = TaskExecutionStats{};

  auto set_failure = [&](const std::string &reason)
  {
    if (out_failure_reason)
      *out_failure_reason = reason;
  };

  auto build_task_debug_summary = [&](int highlight_segment_idx) -> std::string
  {
    std::ostringstream oss;
    oss << "Task execution failure summary";
    if (task.targetObject)
      oss << "\nTarget object: " << task.targetObject->name;
    if (robot)
      oss << "\nAssigned robot: " << robot->name;
    oss << "\nTask start pose (robot): ("
        << std::fixed << std::setprecision(2)
        << task.TaskStartPoseRobot.x << ", "
        << task.TaskStartPoseRobot.y << ", "
        << task.TaskStartPoseRobot.yaw << ")";
    oss << "\nTask goal pose (object): ("
        << task.GoalPoseObj.x << ", "
        << task.GoalPoseObj.y << ", "
        << task.GoalPoseObj.yaw << ")";
    oss << "\nVertex chain:";
    for (const auto &vertex : task.vertexChain)
    {
      oss << " " << vertex.name;
    }
    oss << "\nObs relocation paths: "
        << (task.obsReloPaths ? static_cast<int>(task.obsReloPaths->size()) : 0);
    oss << "\nEdge paths: " << static_cast<int>(task.EdgePaths.size());
    for (std::size_t idx = 0; idx < task.EdgePaths.size(); ++idx)
    {
      const auto &edge = task.EdgePaths[idx];
      oss << "\n  [" << (idx + 1) << "] "
          << ((edge && edge->is_transfer) ? "transfer" : "transit")
          << ", waypoints="
          << (edge ? static_cast<int>(edge->waypoints.size()) : -1);
      if (static_cast<int>(idx + 1) == highlight_segment_idx)
        oss << "  <-- failing segment";
    }
    return oss.str();
  };

  // 1. Plan Transit to Task Start
  double robot_avail_time = timetable.get_entity_max_time(robot) +
                            std::max(0.0, task_start_delay);
  if (out_stats && task_start_delay > 1e-9)
  {
    out_stats->total_waiting += task_start_delay;
  }
  task.TaskStartPoseRobot =
      compute_adjusted_task_start_pose(task, robot, robot_avail_time, timetable);
  std::vector<Waypoint> initial_transit_reference =
      waypoints_from_relopush_state_path(task.firstApproachPath);
  double initial_transit_abs_start = -1.0;
  double initial_transit_abs_end = -1.0;
  if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time, timetable, entities, params, options, &initial_transit_abs_start, &initial_transit_abs_end, [&](double query_time)
                            { return compute_adjusted_task_start_pose(
                                  task, robot, query_time, timetable); }, &initial_transit_reference))
  {
    set_failure("initial transit planning failed");
    std::cerr << "Aborting task due to transit failure." << std::endl;
    return false;
  }
  if (out_stats && initial_transit_abs_start >= 0.0)
  {
    out_stats->has_initial_transit = true;
    out_stats->initial_transit_start_time = initial_transit_abs_start;
    out_stats->initial_transit_end_time = initial_transit_abs_end;
  }
  const Pose initial_wait_pose = timetable.get_pose(robot, initial_transit_abs_end);
  bool initial_wait_gap_checked = false;
  auto record_waiting_pose_conflict = [&](const Pose &wait_pose,
                                          const CollisionInfo &wait_conflict,
                                          const std::string &stage_label,
                                          bool mark_as_initial_gap) -> void
  {
    if (out_stats)
    {
      out_stats->has_waiting_pose_conflict = true;
      out_stats->waiting_pose = wait_pose;
      out_stats->waiting_conflict_time = wait_conflict.time;
      out_stats->waiting_conflict_entity = wait_conflict.entity_name;
      out_stats->waiting_conflict_stage = stage_label;
      if (mark_as_initial_gap)
      {
        out_stats->has_initial_wait_conflict = true;
        out_stats->initial_wait_pose = wait_pose;
        out_stats->initial_wait_conflict_time = wait_conflict.time;
        out_stats->initial_wait_conflict_entity = wait_conflict.entity_name;
      }
    }
  };

  auto validate_wait_gap = [&](const Pose &wait_pose,
                               double from_t,
                               double to_t,
                               const std::string &segment_label,
                               EntityMeta *ignored_entity = nullptr,
                               bool mark_as_initial_gap = false) -> bool
  {
    if (to_t <= from_t + 1e-9)
      return true;

    CollisionInfo wait_conflict = find_robot_waiting_pose_conflict_over_interval(
        robot, wait_pose, from_t, to_t, timetable, params, entities,
        ignored_entity);
    if (wait_conflict.is_valid)
      return true;

    record_waiting_pose_conflict(wait_pose, wait_conflict, segment_label,
                                 mark_as_initial_gap);

    std::ostringstream oss;
    oss << "waiting pose conflicts with reserved occupancy before "
        << segment_label;
    if (!wait_conflict.entity_name.empty())
      oss << " (" << wait_conflict.entity_name << ")";
    set_failure(oss.str());

    std::cerr << "  [Wait] Waiting pose is not safe before "
              << segment_label << ". Conflict: " << wait_conflict.reason;
    if (!wait_conflict.entity_name.empty())
      std::cerr << " with " << wait_conflict.entity_name;
    if (wait_conflict.time > 1e-6)
      std::cerr << " at t=" << std::fixed << std::setprecision(2)
                << wait_conflict.time;
    std::cerr << std::endl;
    return false;
  };

  auto validate_initial_wait_gap = [&](double next_segment_start_time,
                                       const std::string &segment_label,
                                       EntityMeta *ignored_entity = nullptr) -> bool
  {
    if (initial_wait_gap_checked)
      return true;
    initial_wait_gap_checked = true;

    if (next_segment_start_time <= initial_transit_abs_end + 1e-9)
      return true;

    return validate_wait_gap(initial_wait_pose, initial_transit_abs_end,
                             next_segment_start_time,
                             "initial transit -> " + segment_label,
                             ignored_entity, true);
  };

  // 2. ObsRelo (if exists)
  if (task.vertexChain.size() > 2)
  {
    size_t obs_path_base_idx = 0;
    // Iterate through obstacles
    for (size_t obs_ind = 1; obs_ind < task.vertexChain.size() - 1; obs_ind++)
    {
      std::cout << "Obstacle Relocation" << std::endl;

      // Identify the obstacle
      std::string obs_name = task.vertexChain[obs_ind].name;
      auto obs_meta = entities.at(obs_name);

      size_t push_path_idx = obs_path_base_idx;
      size_t post_path_idx = obs_path_base_idx + 1;

      if (task.obsReloPaths->size() > post_path_idx)
      {
        // Step A: Two-point push trajectory
        const double obs_push_wait_start_time =
            timetable.get_entity_max_time(robot, 0.0);
        const Pose obs_push_wait_pose =
            timetable.get_pose(robot, obs_push_wait_start_time);
        double obs_push_start_time = -1.0;
        Trajectory obs_push_traj;
        if (!schedule_path_segment(task.obsReloPaths->at(push_path_idx), obs_meta,
                                   robot, timetable, params, entities, options,
                                   task.sourcePrePushDistance,
                                   nullptr,
                                   transfer_windows,
                                   out_stats, out_failure_reason,
                                   &obs_push_start_time,
                                   &obs_push_traj))
        {
          set_failure("obs relocation push segment failed");
          return false;
        }
        if (!validate_initial_wait_gap(obs_push_start_time,
                                       "obstacle relocation push",
                                       obs_meta))
        {
          return false;
        }
        if (!validate_wait_gap(obs_push_wait_pose, obs_push_wait_start_time,
                               obs_push_start_time,
                               "obstacle relocation push", obs_meta))
        {
          return false;
        }

        if (!append_retraction(robot, obs_push_traj, timetable, params,
                               entities, options, out_stats,
                               out_failure_reason))
        {
          std::cout << "  [Retract] Skipping retraction after obstacle relocation push "
                    << "(no collision-free slot)." << std::endl;
        }

        // Step B: Schedule the "Post-Obs/Return" path
        EntityMeta *return_approach_entity = task.targetObject;
        if (obs_ind + 1 < task.vertexChain.size() - 1)
        {
          auto next_obs_it = entities.find(task.vertexChain[obs_ind + 1].name);
          if (next_obs_it != entities.end())
            return_approach_entity = next_obs_it->second;
        }

        const double obs_return_wait_start_time =
            timetable.get_entity_max_time(robot, 0.0);
        const Pose obs_return_wait_pose =
            timetable.get_pose(robot, obs_return_wait_start_time);
        double obs_return_start_time = -1.0;
        if (!schedule_path_segment(task.obsReloPaths->at(post_path_idx), nullptr,
                                   robot, timetable, params, entities, options,
                                   task.sourcePrePushDistance,
                                   return_approach_entity,
                                   transfer_windows,
                                   out_stats, out_failure_reason,
                                   &obs_return_start_time))
        {
          set_failure("obs relocation return segment failed");
          return false;
        }
        if (!validate_wait_gap(obs_return_wait_pose, obs_return_wait_start_time,
                               obs_return_start_time,
                               "obstacle relocation return"))
        {
          return false;
        }
        obs_path_base_idx += 2;
      }
      else
      {
        std::cerr << "Error: obsReloPaths missing required paths for index "
                  << obs_ind << std::endl;
        return false;
      }
    }
  }

  // 3. Execute Edge Paths (Pushing / Relocation Segments)
  int segment_idx = 0;
  for (auto &path_ptr : task.EdgePaths)
  {
    segment_idx++;
    double segment_ready_time = timetable.get_entity_max_time(robot);
    double segment_wait_start_time = timetable.get_entity_max_time(robot, 0.0);
    Pose segment_wait_pose = timetable.get_pose(robot, segment_wait_start_time);

    // Prepare Trajectory Object
    std::ostringstream fallback_msg;
    fallback_msg << "  [Segment " << segment_idx
                 << "] Transit replanning failed; no MARS-generated segment path accepted.";
    SegmentReplanContext replan_context;
    replan_context.task_id = task_id;
    replan_context.segment_id = segment_idx;
    replan_context.object_name =
        task.targetObject ? task.targetObject->name : "unknown";
    if (!prepare_segment_waypoints_for_scheduling(path_ptr.get(), robot,
                                                  segment_ready_time, timetable,
                                                  entities, params, options,
                                                  fallback_msg.str(),
                                                  replan_context))
    {
      set_failure("segment transit replanning failed");
      std::cerr << " [Error] Segment " << segment_idx
                << " transit replanning failed." << std::endl;
      if (DEBUG_VIS)
      {
        Pose start_pose = timetable.get_pose(robot, segment_ready_time);
        std::ostringstream context;
        context << "Segment " << segment_idx
                << " is empty before scheduling.\n";
        context << build_task_debug_summary(segment_idx);
        visualize_current_state(
            timetable, entities, params, segment_ready_time,
            start_pose, task.GoalPoseObj,
            nullptr, segment_ready_time,
            nullptr, context.str());
      }
      return false;
    }

    Pose segment_goal = path_ptr->waypoints.back();

    // B. Find Valid Start Time (Collision Delay)
    std::cout << "  [Segment " << segment_idx << "] Checking schedule..."
              << std::endl;
    CollisionInfo segment_failure_collision;
    double segment_failure_start = segment_ready_time;
    bool segment_committed = reserve_and_commit_trajectory(
        path_ptr.get(), robot, segment_ready_time, timetable, params,
        entities, options, transfer_windows, out_stats,
        out_failure_reason, &segment_failure_collision,
        &segment_failure_start);

    if (!segment_committed && path_ptr->is_transfer &&
        path_ptr->transferred_object)
    {
      std::string transfer_replan_failure;
      if (replan_transfer_segment_after_failed_schedule(
              path_ptr.get(), robot, segment_ready_time, timetable, entities,
              params, options, &transfer_replan_failure))
      {
        segment_failure_collision =
            CollisionInfo{true, "Not evaluated", "", segment_ready_time};
        segment_failure_start = segment_ready_time;
        segment_committed = reserve_and_commit_trajectory(
            path_ptr.get(), robot, segment_ready_time, timetable, params,
            entities, options, transfer_windows, out_stats,
            out_failure_reason, &segment_failure_collision,
            &segment_failure_start);
      }
      else if (out_failure_reason && !transfer_replan_failure.empty())
      {
        *out_failure_reason = transfer_replan_failure;
      }
    }

    if (!segment_committed)
    {
      std::ostringstream oss;
      oss << "segment " << segment_idx << " failed (no safe start time)";
      set_failure(oss.str());
      std::cerr << " [Error] Segment " << segment_idx
                << " failed (permanent blockage or empty path)" << std::endl;
      if (DEBUG_VIS)
      {
        Pose start_pose = timetable.get_pose(
            robot, segment_ready_time); // Current at ready time

        double debug_query_time = segment_ready_time;
        if (segment_failure_collision.time > 1e-6)
          debug_query_time = segment_failure_collision.time;

        std::ostringstream context;
        context << "Segment " << segment_idx << " scheduling failed. "
                << "Attempted start=" << std::fixed << std::setprecision(2)
                << segment_failure_start << "s";

        visualize_current_state(timetable, entities, params, debug_query_time,
                                start_pose, segment_goal,
                                path_ptr.get(), segment_failure_start,
                                &segment_failure_collision,
                                context.str());
      }
      return false; // Break logic was here, now return false
    }
    EntityMeta *allowed_contact_entity =
        path_ptr->is_transfer ? path_ptr->transferred_object : nullptr;
    if (!validate_initial_wait_gap(path_ptr->start_time,
                                   "segment " + std::to_string(segment_idx),
                                   allowed_contact_entity))
    {
      return false;
    }
    if (!validate_wait_gap(segment_wait_pose, segment_wait_start_time,
                           path_ptr->start_time,
                           "segment " + std::to_string(segment_idx),
                           allowed_contact_entity))
    {
      return false;
    }

    // D. Handle Retraction (if this was a push)
    if (path_ptr->is_transfer)
    {
      if (!append_retraction(robot, *path_ptr, timetable, params, entities,
                             options,
                             out_stats, out_failure_reason))
      {
        std::cout << "  [Retract] Skipping retraction after segment "
                  << segment_idx << " (no collision-free slot)." << std::endl;
      }
    }
  }
  return true;
}

// ==========================================
// 4. MAIN ENTRY POINT
// ==========================================

// Review Index (top-down reading order):
// 1) initialize_params / initialize_entities: environment and entity setup.
// 2) get_sorted_candidate_robots: earliest-ready + RS-biased candidate order.
// 3) plan_initial_transit: first approach planning with fallback layers.
// 4) prepare_segment_waypoints_for_scheduling: normalize segment waypoints.
// 5) find_safe_start_time: wait/relocate/retry conflict-resolution scheduler.
// 6) schedule_path_segment: segment-level scheduling and registration.
// 7) process_task_execution: full task pipeline over all edge segments.
// 8) append_retraction: optional post-transfer backing motion.
// 9) verify_timetable_collision_free: post-allocation global safety check.

std::string default_sequence_path()
{
  return std::string(CMAKE_SOURCE_DIR) +
         "/result_seq_ReloPush-BOSS_8_objects.txt_ind5.b64";
}

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

  loaded_sequence = loadFinalSequenceFromFile(filename);
  if (loaded_sequence.empty())
  {
    std::cerr << "[System] Failed to load sequence." << std::endl;
    return false;
  }

  return true;
}

RuntimeOptions parse_runtime_options(int argc, char **argv)
{
  RuntimeOptions options;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i] ? argv[i] : "";
    if (arg == "--robot-boundary-mode=origin" ||
        arg == "--robot-boundary-origin-only")
    {
      options.robot_boundary_origin_only = true;
    }
    else if (arg == "--robot-boundary-mode=corners")
    {
      options.robot_boundary_origin_only = false;
    }
    else if (arg == "--parking-candidate-mode=expand")
    {
      options.parking_candidate_mode = ParkingCandidateMode::EXPAND;
    }
    else if (arg == "--parking-candidate-mode=connected")
    {
      options.parking_candidate_mode = ParkingCandidateMode::CONNECTED;
    }
    else if (arg == "--parking-candidate-mode=vfh" ||
             arg == "--parking-candidate-mode=connected-vfh")
    {
      options.parking_candidate_mode = ParkingCandidateMode::CONNECTED_VFH;
    }
    else if (arg == "--parking-candidate-mode=reverse-recent" ||
             arg == "--parking-candidate-mode=reverse-path" ||
             arg == "--parking-candidate-mode=retrace")
    {
      options.parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT;
    }
    else if (arg == "--parking-candidate-mode=reverse-recent-shorter" ||
             arg == "--parking-candidate-mode=reverse-recent-refined" ||
             arg == "--parking-candidate-mode=reverse-shorter" ||
             arg == "--parking-candidate-mode=retrace-shorter")
    {
      options.parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT_SHORTER;
    }
    else if (arg == "--parking-candidate-mode=random")
    {
      options.parking_candidate_mode = ParkingCandidateMode::RANDOM;
    }
    else if (arg == "--enable-order-learning" ||
             arg == "--order-learning")
    {
      options.enable_order_constraint_learning = true;
    }
    else if (arg == "--disable-order-learning" ||
             arg == "--no-order-learning")
    {
      options.enable_order_constraint_learning = false;
    }
    else if (arg.rfind("--random-seed=", 0) == 0)
    {
      std::string seed_text = arg.substr(std::string("--random-seed=").size());
      try
      {
        unsigned long parsed = std::stoul(seed_text);
        options.base_random_seed = static_cast<std::uint32_t>(parsed);
        options.has_fixed_random_seed = true;
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --random-seed value: '" << seed_text
                  << "'. Falling back to non-deterministic seed." << std::endl;
      }
    }
    else if (arg.rfind("--lns-threads=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--lns-threads=").size());
      try
      {
        options.lns_threads = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --lns-threads value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--planner-expansion-threads=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--planner-expansion-threads=").size());
      try
      {
        options.planner_expansion_threads = std::max(1, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --planner-expansion-threads value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--max-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--max-search-iters=").size());
      try
      {
        options.max_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --max-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--assignment-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--assignment-search-iters=").size());
      try
      {
        options.assignment_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --assignment-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--sequence-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--sequence-search-iters=").size());
      try
      {
        int parsed_iters = std::max(0, std::stoi(value));
        options.local_sequence_search_iterations = parsed_iters;
        options.shuffle_sequence_search_iterations = parsed_iters;
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --sequence-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--local-sequence-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--local-sequence-search-iters=").size());
      try
      {
        options.local_sequence_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --local-sequence-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--shuffle-sequence-search-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--shuffle-sequence-search-iters=").size());
      try
      {
        options.shuffle_sequence_search_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --shuffle-sequence-search-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--lns-iters=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--lns-iters=").size());
      try
      {
        options.lns_iterations = std::max(0, std::stoi(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --lns-iters value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg == "--lns-fine-segment-retry")
    {
      options.enable_lns_fine_segment_retry = true;
    }
    else if (arg == "--no-lns-fine-segment-retry")
    {
      options.enable_lns_fine_segment_retry = false;
    }
    else if (arg == "--initial-transit-fallbacks")
    {
      options.enable_initial_transit_fallbacks = true;
    }
    else if (arg == "--no-initial-transit-fallbacks")
    {
      options.enable_initial_transit_fallbacks = false;
    }
    else if (arg == "--reference-egraph-transit")
    {
      options.enable_reference_egraph_transit = true;
    }
    else if (arg == "--no-reference-egraph-transit")
    {
      options.enable_reference_egraph_transit = false;
    }
    else if (arg == "--visualize" || arg == "--visualization")
    {
      options.enable_visualization = true;
    }
    else if (arg == "--no-visualization")
    {
      options.enable_visualization = false;
    }
    else if (arg == "--visualize-relopush-plan" ||
             arg == "--debug-relopush-plan")
    {
      options.visualize_relopush_plan = true;
    }
    else if (arg == "--no-visualize-relopush-plan")
    {
      options.visualize_relopush_plan = false;
    }
    else if (arg == "--result-summary-figure" ||
             arg == "--visualize-result-summary" ||
             arg == "--visualize-result-traces")
    {
      options.enable_result_summary_figure = true;
    }
    else if (arg == "--no-result-summary-figure")
    {
      options.enable_result_summary_figure = false;
    }
    else if (arg.rfind("--result-summary-output=", 0) == 0)
    {
      options.result_summary_output_path =
          arg.substr(std::string("--result-summary-output=").size());
      options.enable_result_summary_figure = true;
    }
    else if (arg.rfind("--result-summary-gap=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--result-summary-gap=").size());
      try
      {
        options.result_summary_subplot_gap = std::max(0.0, std::stod(value));
      }
      catch (...)
      {
        std::cerr << "[Warn] Invalid --result-summary-gap value: '" << value
                  << "'. Keeping default." << std::endl;
      }
    }
    else if (arg.rfind("--robot-trace-colors=", 0) == 0)
    {
      std::string value = arg.substr(std::string("--robot-trace-colors=").size());
      std::vector<std::string> parsed_colors;
      std::stringstream ss(value);
      std::string color;
      while (std::getline(ss, color, ','))
      {
        color.erase(std::remove_if(color.begin(), color.end(),
                                   [](unsigned char ch)
                                   { return std::isspace(ch); }),
                    color.end());
        if (!color.empty())
        {
          parsed_colors.push_back(color);
        }
      }
      if (!parsed_colors.empty())
      {
        options.robot_trace_colors = parsed_colors;
      }
    }
    else if (arg == "--integrated-mode" ||
             arg == "--integrated")
    {
      options.integrated_mode = true;
    }
    else if (arg.rfind("--handoff-endpoint=", 0) == 0)
    {
      options.handoff_endpoint = arg.substr(std::string("--handoff-endpoint=").size());
    }
    else if (arg.rfind("--mars-endpoint=", 0) == 0)
    {
      options.handoff_endpoint = arg.substr(std::string("--mars-endpoint=").size());
    }
    else if (arg.rfind("--sequence-file=", 0) == 0)
    {
      options.input_sequence_path = arg.substr(std::string("--sequence-file=").size());
    }
    else if (arg.rfind("--input-sequence=", 0) == 0)
    {
      options.input_sequence_path = arg.substr(std::string("--input-sequence=").size());
    }
    else if (arg == "--debug-vis")
    {
      options.debug_vis = true;
    }
    else if (arg == "--no-debug-vis")
    {
      options.debug_vis = false;
    }
  }

  if (!options.has_fixed_random_seed)
  {
    std::random_device rd;
    options.base_random_seed = rd();
  }

  return options;
}

void print_runtime_options(const RuntimeOptions &options)
{
  std::cout << "[Config] Debug vis logs: "
            << (options.debug_vis ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Robot boundary mode: "
            << (options.robot_boundary_origin_only ? "origin-only" : "corner-strict")
            << std::endl;
  std::cout << "[Config] Base RNG seed: " << options.base_random_seed
            << (options.has_fixed_random_seed ? " (fixed)" : " (randomized)")
            << std::endl;
  std::cout << "[Config] LNS threads: "
            << options.lns_threads << std::endl;
  std::cout << "[Config] Planner expansion threads: "
            << options.planner_expansion_threads << std::endl;
  if (options.lns_threads > 1 && options.planner_expansion_threads > 1)
  {
    std::cout << "[Config] Planner expansion threads are disabled inside "
                 "parallel LNS/search batches to avoid nested oversubscription."
              << std::endl;
  }
  std::cout << "[Config] Max search iterations: "
            << options.max_search_iterations << std::endl;
  std::cout << "[Config] Default search params: "
            << "xy=" << options.default_xy_resolution
            << ", yaw=" << options.default_yaw_resolution
            << ", time_step=" << options.default_time_step
            << ", rs_step=" << options.default_rs_step_size
            << ", collision_steps=" << options.default_collision_steps
            << ", collision_step="
            << options.default_collision_check_time_step
            << ", analytic_scale="
            << options.default_analytic_threshold_scale << std::endl;
  std::cout << "[Config] Default search costs/collision: "
            << "turn_penalty=" << options.default_turn_penalty
            << ", reverse_penalty=" << options.default_reverse_penalty
            << ", switch_penalty=" << options.default_switch_penalty
            << ", wait_penalty=" << options.default_wait_penalty
            << ", inflation=" << options.default_inflation
            << ", safety_margin=" << options.default_safety_margin
            << ", robot_collision_inflation="
            << options.default_robot_collision_inflation << std::endl;
  std::cout << "[Config] Fine segment retry params: "
            << "max_iter=" << options.fine_segment_max_search_iterations
            << ", xy=" << options.fine_segment_xy_resolution
            << ", yaw=" << options.fine_segment_yaw_resolution
            << ", time_step=" << options.fine_segment_time_step
            << ", rs_step=" << options.fine_segment_rs_step_size
            << ", collision_step="
            << options.fine_segment_collision_check_time_step << std::endl;
  std::cout << "[Config] Parking candidate mode: "
            << parking_candidate_mode_name(options.parking_candidate_mode)
            << std::endl;
  std::cout << "[Config] Order-constraint learning: "
            << (options.enable_order_constraint_learning ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Assignment search iterations: "
            << options.assignment_search_iterations << std::endl;
  std::cout << "[Config] Local sequence search iterations: "
            << options.local_sequence_search_iterations << std::endl;
  std::cout << "[Config] Shuffle sequence search iterations: "
            << options.shuffle_sequence_search_iterations << std::endl;
  std::cout << "[Config] LNS iterations: "
            << options.lns_iterations << std::endl;
  std::cout << "[Config] LNS fine segment retry: "
            << (options.enable_lns_fine_segment_retry ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Initial transit fallbacks: "
            << (options.enable_initial_transit_fallbacks ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Reference E-Graph transit: "
            << (options.enable_reference_egraph_transit ? "enabled" : "disabled")
            << ", epsilon=" << options.reference_egraph_epsilon
            << ", spacing=" << options.reference_egraph_waypoint_spacing
            << ", snap_radius=" << options.reference_egraph_snap_radius
            << ", snap_yaw=" << options.reference_egraph_snap_yaw
            << ", lookahead=" << options.reference_egraph_successor_lookahead
            << ", max_nodes=" << options.reference_egraph_max_nodes
            << std::endl;
  std::cout << "[Config] Visualization: "
            << (options.enable_visualization ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] ReloPush plan replay visualization: "
            << (options.visualize_relopush_plan ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Result summary figure: "
            << (options.enable_result_summary_figure ? "enabled" : "disabled");
  if (options.enable_result_summary_figure)
  {
    std::cout << ", gap=" << options.result_summary_subplot_gap << "px";
    if (!options.result_summary_output_path.empty())
      std::cout << ", output=" << options.result_summary_output_path;
  }
  std::cout << std::endl;
  std::cout << "[Config] Input mode: "
            << (options.integrated_mode ? "integrated-handoff" : "sequence-file")
            << std::endl;
  if (options.integrated_mode)
  {
    std::cout << "[Config] Handoff endpoint: "
              << options.handoff_endpoint << std::endl;
  }
  else
  {
    std::cout << "[Config] Sequence file: "
              << (options.input_sequence_path.empty()
                      ? default_sequence_path()
                      : options.input_sequence_path)
              << std::endl;
  }
}

void initialize_environment(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    std::uint32_t parking_seed,
    Params &params,
    std::unordered_map<std::string, EntityMeta *> &entities,
    TimeTable &timetable,
    std::vector<RobotMeta *> &all_robots,
    bool verbose)
{
  reset_thread_local_planning_state();
  params = initialize_params(loaded_sequence, options);
  params.robot_boundary_origin_only = options.robot_boundary_origin_only;
  g_parking_candidate_mode = options.parking_candidate_mode;
  initialize_parking_rng(true, parking_seed);

  if (verbose)
  {
    std::cout << "[Config] Parking RNG seed: " << parking_rng_seed() << std::endl;
    std::cout << "[Config] Parking candidate mode: "
              << parking_candidate_mode_name() << std::endl;
    std::ostringstream collision_tuning;
    collision_tuning << std::fixed << std::setprecision(3)
                     << "[Config] Collision tuning: inflation="
                     << params.inflation
                     << ", safety_margin=" << params.safety_margin
                     << ", robot_collision_inflation="
                     << params.robot_collision_inflation
                     << " (temporary experiment)";
    std::cout << collision_tuning.str() << std::endl;
  }

  entities = initialize_entities(loaded_sequence);
  timetable.add_initial(entities);

  all_robots.clear();
  for (const auto &[name, ent] : entities)
  {
    if (ent->type == EntityType::ROBOT)
      all_robots.push_back(dynamic_cast<RobotMeta *>(ent));
  }
  std::sort(all_robots.begin(), all_robots.end(),
            [](const RobotMeta *a, const RobotMeta *b)
            {
              return a->name < b->name;
            });
  if (verbose && !all_robots.empty())
  {
    const RobotMeta *sample_robot = all_robots.front();
    std::cout << std::fixed << std::setprecision(3)
              << "[Config] Robot turning radii: transit="
              << sample_robot->transit_turning_radius()
              << "m, transfer="
              << sample_robot->transfer_turning_radius()
              << "m" << std::endl;
  }
}

std::vector<Task> initialize_tasks(
    const std::vector<FinalAllocation> &loaded_sequence,
    std::unordered_map<std::string, EntityMeta *> &entities)
{
  std::vector<Task> tasks;
  tasks.reserve(loaded_sequence.size());
  for (const auto &fa : loaded_sequence)
  {
    tasks.emplace_back(fa, entities);
  }

  std::cout << "[System] Initialized " << tasks.size() << " tasks."
            << std::endl;
  return tasks;
}

AllocationScenarioPlan make_identity_plan(std::size_t task_count)
{
  AllocationScenarioPlan plan;
  plan.task_order.resize(task_count);
  std::iota(plan.task_order.begin(), plan.task_order.end(), 0);
  return plan;
}

std::unordered_map<std::string, RobotMeta *>
build_robot_lookup(const std::vector<RobotMeta *> &all_robots)
{
  std::unordered_map<std::string, RobotMeta *> lookup;
  for (auto *robot : all_robots)
  {
    if (robot)
      lookup[robot->name] = robot;
  }
  return lookup;
}

std::vector<std::size_t> normalized_task_order(
    const AllocationScenarioPlan &plan,
    std::size_t task_count)
{
  if (plan.task_order.size() != task_count)
  {
    auto identity_plan = make_identity_plan(task_count);
    return identity_plan.task_order;
  }

  std::vector<bool> seen(task_count, false);
  for (std::size_t idx : plan.task_order)
  {
    if (idx >= task_count || seen[idx])
    {
      auto identity_plan = make_identity_plan(task_count);
      return identity_plan.task_order;
    }
    seen[idx] = true;
  }

  return plan.task_order;
}

std::vector<Task> build_tasks_for_plan(
    const std::vector<FinalAllocation> &loaded_sequence,
    std::unordered_map<std::string, EntityMeta *> &entities,
    const std::vector<RobotMeta *> &all_robots,
    const AllocationScenarioPlan &plan)
{
  auto base_tasks = initialize_tasks(loaded_sequence, entities);
  auto order = normalized_task_order(plan, base_tasks.size());
  auto robot_lookup = build_robot_lookup(all_robots);

  std::vector<Task> ordered_tasks;
  ordered_tasks.reserve(base_tasks.size());

  for (std::size_t original_idx : order)
  {
    ordered_tasks.push_back(base_tasks[original_idx]);
    Task &task = ordered_tasks.back();
    task.assignedRobot = nullptr;

    if (original_idx < plan.preferred_robot_names_by_original_task.size())
    {
      const auto &preferred_name =
          plan.preferred_robot_names_by_original_task[original_idx];
      auto it = robot_lookup.find(preferred_name);
      if (!preferred_name.empty() && it != robot_lookup.end())
      {
        task.assignedRobot = it->second;
      }
    }
  }

  return ordered_tasks;
}

std::vector<std::pair<RobotMeta *, double>>
prepare_task_candidates(Task &task,
                        const std::vector<RobotMeta *> &all_robots,
                        TimeTable &timetable,
                        const Params &params)
{
  auto candidates =
      get_sorted_candidate_robots(all_robots, timetable, task, params);

  if (task.assignedRobot)
  {
    auto it = std::find_if(candidates.begin(), candidates.end(),
                           [&](const auto &p)
                           { return p.first == task.assignedRobot; });
    if (it != candidates.end())
    {
      std::rotate(candidates.begin(), it, it + 1);
    }
  }

  return candidates;
}

TaskCsvRow initialize_task_row(
    int task_id,
    const Task &task,
    const std::vector<std::pair<RobotMeta *, double>> &candidates)
{
  TaskCsvRow row;
  row.task_id = task_id;
  row.object_name = task.targetObject ? task.targetObject->name : "unknown";
  row.status = "FAILED";
  row.robot_name = "";
  row.start_time = candidates.empty() ? -1.0 : candidates.front().second;
  row.end_time = -1.0;
  row.total_waiting = 0.0;
  row.attempts = 0;
  row.failure_reason = "";
  return row;
}

bool attempt_task_with_candidate(
    Task &task,
    RobotMeta *cand_robot,
    double free_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> &transfer_windows,
    TaskCsvRow &row,
    std::string &last_failed_robot,
    std::string &last_failure_reason)
{
  TimeTable timetable_before_attempt = timetable;
  auto transfer_windows_before_attempt = transfer_windows;
  TimeTable retry_base_timetable = timetable_before_attempt;
  auto retry_base_transfer_windows = transfer_windows_before_attempt;
  task.assignedRobot = cand_robot;
  std::cout << "[Assign] Attempting " << cand_robot->name
            << " (Free at t=" << std::fixed << std::setprecision(2) << free_time << "s)" << std::endl;

  constexpr double kPostValidationRetryDelay = 0.5;
  constexpr int kMaxPostValidationDelayRetries = 4;
  double task_start_delay = 0.0;
  bool self_safe_parking_retry_applied = false;

  auto make_single_pose_hint = [&](const Pose &pose, double abs_time) -> Trajectory
  {
    Trajectory hint;
    hint.entity = cand_robot;
    hint.start_time = abs_time;
    hint.is_transfer = false;

    Waypoint wp;
    wp.x = pose.x;
    wp.y = pose.y;
    wp.yaw = pose.yaw;
    wp.time = 0.0;
    wp.linear_velocity = 0.0;
    wp.steering_angle = 0.0;
    hint.waypoints.push_back(wp);
    return hint;
  };

  for (int retry_idx = 0; retry_idx <= kMaxPostValidationDelayRetries; ++retry_idx)
  {
    timetable = retry_base_timetable;
    transfer_windows = retry_base_transfer_windows;

    if (retry_idx > 0)
    {
      std::cout << "         Retrying " << cand_robot->name
                << " with extra task-start delay of "
                << std::fixed << std::setprecision(2)
                << task_start_delay << "s." << std::endl;
    }

    TaskExecutionStats attempt_stats;
    std::string attempt_failure_reason;
    if (process_task_execution(cand_robot, task, timetable, entities, params,
                               options,
                               &transfer_windows,
                               &attempt_stats, &attempt_failure_reason,
                               task_start_delay, row.task_id))
    {
      auto verify = verify_timetable_collision_free(timetable, entities, params,
                                                    transfer_windows,
                                                    0.0);
      if (!verify.is_valid)
      {
        std::ostringstream vmsg;
        vmsg << "post-task verification collision at t="
             << std::fixed << std::setprecision(2)
             << verify.time << " between " << verify.entity_a
             << " and " << verify.entity_b;
        attempt_failure_reason = vmsg.str();

        EntityMeta *entity_a = nullptr;
        EntityMeta *entity_b = nullptr;
        auto entity_a_it = entities.find(verify.entity_a);
        if (entity_a_it != entities.end())
          entity_a = entity_a_it->second;
        auto entity_b_it = entities.find(verify.entity_b);
        if (entity_b_it != entities.end())
          entity_b = entity_b_it->second;

        const bool candidate_involved =
            (verify.entity_a == cand_robot->name ||
             verify.entity_b == cand_robot->name);
        const std::string other_entity_name =
            (verify.entity_a == cand_robot->name)
                ? verify.entity_b
                : ((verify.entity_b == cand_robot->name) ? verify.entity_a
                                                         : "");
        const bool collision_during_initial_transit =
            attempt_stats.has_initial_transit &&
            candidate_involved &&
            verify.time + 1e-6 >= attempt_stats.initial_transit_start_time &&
            verify.time <= attempt_stats.initial_transit_end_time + 1e-6;
        const bool later_transit_priority_inversion =
            collision_during_initial_transit &&
            !other_entity_name.empty() &&
            (!task.targetObject || other_entity_name != task.targetObject->name);

        std::cout << "         [VerifyDebug] "
                  << describe_entity_pose_context(timetable, entity_a, verify.time)
                  << std::endl;
        std::cout << "         [VerifyDebug] "
                  << describe_entity_pose_context(timetable, entity_b, verify.time)
                  << std::endl;
        std::cout << "         [VerifyDebug] "
                  << summarize_pair_collision_window(timetable, entity_a, entity_b,
                                                     params, verify.time)
                  << std::endl;
        std::cout << "         [VerifyDebug] "
                  << compare_pair_collision_checks(timetable, entity_a, entity_b,
                                                   params, verify.time)
                  << std::endl;

        if (DEBUG_VIS)
        {
          std::ostringstream context;
          context << "Task " << row.task_id;
          if (task.targetObject)
            context << " (" << task.targetObject->name << ")";
          context << "\nCandidate robot: " << cand_robot->name;
          context << "\nVerification result recorded after successful task execution";
          visualize_post_task_verification_debug(
              timetable, cand_robot, task.targetObject,
              verify.time, verify.reason, verify.entity_a, verify.entity_b, params,
              context.str());
        }

        if (later_transit_priority_inversion &&
            retry_idx < kMaxPostValidationDelayRetries)
        {
          task_start_delay += kPostValidationRetryDelay;
          std::cout << "         Earlier reserved occupancy keeps priority over "
                    << cand_robot->name
                    << "'s later initial transit. Adding "
                    << std::fixed << std::setprecision(2)
                    << kPostValidationRetryDelay
                    << "s task-start delay and retrying the same robot."
                    << std::endl;
          continue;
        }

        if (later_transit_priority_inversion &&
            !self_safe_parking_retry_applied)
        {
          TimeTable self_park_base_timetable = retry_base_timetable;
          auto self_park_base_transfer_windows = retry_base_transfer_windows;
          Pose conflict_pose = timetable.get_pose(cand_robot, verify.time);
          Trajectory blocked_hint = make_single_pose_hint(conflict_pose, verify.time);

          std::cout << "         Earlier relocation kept priority after delay retries."
                    << " Attempting self safe parking for " << cand_robot->name
                    << " before replanning the task." << std::endl;

          if (relocate_blocking_robot(cand_robot, self_park_base_timetable,
                                      params, entities, options, &blocked_hint))
          {
            retry_base_timetable = std::move(self_park_base_timetable);
            retry_base_transfer_windows = std::move(self_park_base_transfer_windows);
            self_safe_parking_retry_applied = true;
            task_start_delay = 0.0;
            retry_idx = -1;
            std::cout << "         " << cand_robot->name
                      << " reached safe parking. Re-running the same task while preserving earlier reservations."
                      << std::endl;
            continue;
          }

          std::cout << "         Self safe parking retry failed for "
                    << cand_robot->name << "." << std::endl;
        }

        if (!later_transit_priority_inversion &&
            retry_idx < kMaxPostValidationDelayRetries)
        {
          task_start_delay += kPostValidationRetryDelay;
          std::cout << "         Verification collision remained after planning."
                    << " Adding " << std::fixed << std::setprecision(2)
                    << kPostValidationRetryDelay
                    << "s more task-start delay and retrying the same robot."
                    << std::endl;
          continue;
        }

        timetable = timetable_before_attempt;
        transfer_windows = transfer_windows_before_attempt;
        last_failed_robot = cand_robot->name;
        last_failure_reason = attempt_failure_reason;
        std::cout << "[Assign] FAILED with " << cand_robot->name
                  << " (Post-Task Verification Failed) - Trying next..."
                  << std::endl;
        std::cout << "         Verification: collision at t="
                  << std::fixed << std::setprecision(2) << verify.time
                  << " between " << verify.entity_a
                  << " and " << verify.entity_b << std::endl;
        return false;
      }

      row.status = "SUCCESS";
      row.robot_name = cand_robot->name;
      row.start_time = attempt_stats.has_initial_transit
                           ? attempt_stats.initial_transit_start_time
                           : (free_time + task_start_delay);
      row.end_time = timetable.get_entity_max_time(cand_robot);
      row.total_waiting = attempt_stats.total_waiting;
      row.failure_reason = "";
      std::cout << "[Assign] SUCCESS with " << cand_robot->name << std::endl;
      return true;
    }

    if (attempt_stats.has_waiting_pose_conflict &&
        retry_idx < kMaxPostValidationDelayRetries)
    {
      task_start_delay += kPostValidationRetryDelay;
      std::cout << "         Waiting pose conflicts with earlier reserved occupancy";
      if (!attempt_stats.waiting_conflict_stage.empty())
        std::cout << " before " << attempt_stats.waiting_conflict_stage;
      std::cout
          << "."
          << " Adding " << std::fixed << std::setprecision(2)
          << kPostValidationRetryDelay
          << "s task-start delay and retrying the same robot."
          << std::endl;
      continue;
    }

    if (attempt_stats.has_waiting_pose_conflict &&
        !self_safe_parking_retry_applied)
    {
      TimeTable self_park_base_timetable = retry_base_timetable;
      auto self_park_base_transfer_windows = retry_base_transfer_windows;
      const double conflict_time =
          attempt_stats.waiting_conflict_time > 0.0
              ? attempt_stats.waiting_conflict_time
              : attempt_stats.initial_transit_end_time;
      Trajectory blocked_hint =
          make_single_pose_hint(attempt_stats.waiting_pose, conflict_time);

      std::cout << "         Delay retries exhausted while "
                << cand_robot->name
                << " waits before continuing the task."
                << (attempt_stats.waiting_conflict_stage.empty()
                        ? ""
                        : (" Problem stage: " +
                           attempt_stats.waiting_conflict_stage + "."))
                << " Attempting self safe parking before replanning the task."
                << std::endl;

      if (relocate_blocking_robot(cand_robot, self_park_base_timetable,
                                  params, entities, options, &blocked_hint))
      {
        retry_base_timetable = std::move(self_park_base_timetable);
        retry_base_transfer_windows = std::move(self_park_base_transfer_windows);
        self_safe_parking_retry_applied = true;
        task_start_delay = 0.0;
        retry_idx = -1;
        std::cout << "         " << cand_robot->name
                  << " reached safe parking. Re-running the same task while preserving earlier reservations."
                  << std::endl;
        continue;
      }

      std::cout << "         Self safe parking retry failed for "
                << cand_robot->name << "." << std::endl;
    }

    timetable = timetable_before_attempt;
    transfer_windows = transfer_windows_before_attempt;
    last_failed_robot = cand_robot->name;
    last_failure_reason = attempt_failure_reason.empty() ? "task execution failed" : attempt_failure_reason;
    std::cout << "[Assign] FAILED with " << cand_robot->name << " (Transit Blocked) - Trying next..." << std::endl;
    if (!last_failure_reason.empty())
    {
      std::cout << "         Reason: " << last_failure_reason << std::endl;
    }
    return false;
  }

  timetable = timetable_before_attempt;
  transfer_windows = transfer_windows_before_attempt;
  last_failed_robot = cand_robot->name;
  last_failure_reason = "unexpected exhausted retry loop";
  return false;
}

TaskCsvRow execute_single_task_with_candidates(
    Task &task,
    int task_counter,
    const std::vector<RobotMeta *> &all_robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> &transfer_windows)
{
  std::cout << "\n=== Processing Task " << task_counter << " ("
            << task.targetObject->name << ") ===" << std::endl;

  auto candidates = prepare_task_candidates(task, all_robots, timetable, params);
  TaskCsvRow row = initialize_task_row(task_counter, task, candidates);

  bool task_success = false;
  std::string last_failed_robot = "";
  std::string last_failure_reason = "";

  for (auto &[cand_robot, free_time] : candidates)
  {
    row.attempts += 1;
    if (attempt_task_with_candidate(task, cand_robot, free_time,
                                    timetable, entities, params, options,
                                    transfer_windows, row,
                                    last_failed_robot, last_failure_reason))
    {
      task_success = true;
      break;
    }
  }

  if (!task_success)
  {
    row.status = "FAILED";
    row.robot_name = last_failed_robot;
    row.failure_reason = last_failure_reason.empty() ? "all candidate robots failed" : last_failure_reason;
    std::cerr << "[Critical] Task " << task_counter << " failed with ALL available robots." << std::endl;
  }

  return row;
}

std::vector<TaskCsvRow> execute_task_allocation_loop(
    std::vector<Task> &tasks,
    const std::vector<RobotMeta *> &all_robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options)
{
  std::vector<TaskCsvRow> task_rows;
  std::vector<TransferContactWindow> transfer_windows;

  int task_counter = 0;
  for (auto &task : tasks)
  {
    task_counter++;
    auto row = execute_single_task_with_candidates(
        task, task_counter, all_robots, timetable, entities, params,
        options,
        transfer_windows);
    task_rows.push_back(std::move(row));
  }

  return task_rows;
}

AllocationRunSummary summarize_run(
    const std::string &label,
    const AllocationScenarioPlan &plan,
    std::uint32_t parking_seed,
    const std::vector<TaskCsvRow> &task_rows,
    const TimeTable &timetable)
{
  AllocationRunSummary summary;
  summary.label = label;
  summary.plan = plan;
  summary.parking_seed = parking_seed;
  summary.task_rows = task_rows;
  summary.successful_tasks = static_cast<int>(
      std::count_if(task_rows.begin(), task_rows.end(),
                    [](const TaskCsvRow &row)
                    { return row.status == "SUCCESS"; }));
  summary.failed_tasks = static_cast<int>(task_rows.size()) - summary.successful_tasks;
  summary.all_tasks_succeeded = summary.failed_tasks == 0;
  summary.makespan = summary.all_tasks_succeeded
                         ? timetable.get_max_time()
                         : std::numeric_limits<double>::infinity();
  return summary;
}

ExecutedScenario execute_allocation_scenario(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &plan,
    const std::string &label,
    std::uint32_t parking_seed,
    bool verbose)
{
  ScopedStreamSilencer silencer(!verbose);

  ExecutedScenario executed;
  std::vector<RobotMeta *> all_robots;
  initialize_environment(loaded_sequence, options, parking_seed,
                         executed.params, executed.entities,
                         executed.timetable, all_robots, verbose);

  auto tasks = build_tasks_for_plan(loaded_sequence, executed.entities,
                                    all_robots, plan);
  auto task_rows = execute_task_allocation_loop(
      tasks, all_robots, executed.timetable, executed.entities, executed.params,
      options);

  executed.summary = summarize_run(label, plan, parking_seed, task_rows,
                                   executed.timetable);
  return executed;
}

std::vector<ScenarioEvaluationResult> evaluate_scenario_batch(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const std::vector<ScenarioEvaluationRequest> &requests)
{
  std::vector<ScenarioEvaluationResult> results(requests.size());
  if (requests.empty())
    return results;

  std::size_t worker_count = std::min<std::size_t>(
      requests.size(),
      static_cast<std::size_t>(std::max(1, options.lns_threads)));

  if (worker_count <= 1)
  {
    for (std::size_t i = 0; i < requests.size(); ++i)
    {
      auto executed = execute_allocation_scenario(
          loaded_sequence, options, requests[i].plan, requests[i].label,
          requests[i].parking_seed, false);
      results[i].summary = std::move(executed.summary);
    }
    return results;
  }

  ScopedGlobalStreamSilencer silencer(true);
  RuntimeOptions worker_options = options;
  worker_options.planner_expansion_threads = 1;
  std::atomic<std::size_t> next_index{0};
  std::vector<std::thread> workers;
  workers.reserve(worker_count);

  for (std::size_t worker_idx = 0; worker_idx < worker_count; ++worker_idx)
  {
    workers.emplace_back(
        [&loaded_sequence, &worker_options, &requests, &results, &next_index]()
        {
          while (true)
          {
            std::size_t idx = next_index.fetch_add(1);
            if (idx >= requests.size())
              break;

            auto executed = execute_allocation_scenario(
                loaded_sequence, worker_options, requests[idx].plan, requests[idx].label,
                requests[idx].parking_seed, true);
            results[idx].summary = std::move(executed.summary);
          }
        });
  }

  for (auto &worker : workers)
  {
    if (worker.joinable())
      worker.join();
  }

  return results;
}

AllocationRunSummary repair_destroyed_tasks_with_sampled_insertion(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<std::size_t> &destroyed_tasks,
    const std::vector<std::string> &robot_names,
    const std::vector<double> &destroy_scores,
    std::uint32_t parking_seed,
    std::uint32_t repair_seed,
    const std::string &label,
    bool disable_local_silencer = false);

std::vector<AllocationRunSummary> evaluate_lns_batch(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<LnsSearchCandidate> &candidates,
    const std::vector<std::string> &robot_names,
    const std::string &label)
{
  std::vector<AllocationRunSummary> results(candidates.size());
  if (candidates.empty())
    return results;

  std::size_t worker_count = std::min<std::size_t>(
      candidates.size(),
      static_cast<std::size_t>(std::max(1, options.lns_threads)));

  if (worker_count <= 1)
  {
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      results[i] = repair_destroyed_tasks_with_sampled_insertion(
          loaded_sequence, options, base_plan, base_summary, constraints,
          candidates[i].destroyed_tasks, robot_names, candidates[i].destroy_scores,
          candidates[i].parking_seed, candidates[i].repair_seed, label);
    }
    return results;
  }

  ScopedGlobalStreamSilencer silencer(true);
  RuntimeOptions worker_options = options;
  worker_options.planner_expansion_threads = 1;
  std::atomic<std::size_t> next_index{0};
  std::vector<std::thread> workers;
  workers.reserve(worker_count);

  for (std::size_t worker_idx = 0; worker_idx < worker_count; ++worker_idx)
  {
    workers.emplace_back(
        [&loaded_sequence, &worker_options, &base_plan, &base_summary, &constraints,
         &candidates, &results, &robot_names, &label, &next_index]()
        {
          while (true)
          {
            std::size_t idx = next_index.fetch_add(1);
            if (idx >= candidates.size())
              break;

            results[idx] = repair_destroyed_tasks_with_sampled_insertion(
                loaded_sequence, worker_options, base_plan, base_summary, constraints,
                candidates[idx].destroyed_tasks, robot_names,
                candidates[idx].destroy_scores, candidates[idx].parking_seed,
                candidates[idx].repair_seed, label, true);
          }
        });
  }

  for (auto &worker : workers)
  {
    if (worker.joinable())
      worker.join();
  }

  return results;
}

AllocationScenarioPlan make_assignment_plan_from_greedy(
    const AllocationRunSummary &greedy_summary)
{
  AllocationScenarioPlan plan = greedy_summary.plan;
  plan.preferred_robot_names_by_original_task.assign(
      greedy_summary.task_rows.size(), "");

  for (const auto &row : greedy_summary.task_rows)
  {
    if (row.task_id <= 0)
      continue;
    std::size_t idx = static_cast<std::size_t>(row.task_id - 1);
    if (idx < plan.preferred_robot_names_by_original_task.size() &&
        row.status == "SUCCESS")
    {
      plan.preferred_robot_names_by_original_task[idx] = row.robot_name;
    }
  }

  return plan;
}

AllocationScenarioPlan mutate_assignment_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::string> &robot_names,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  std::size_t task_count = candidate.task_order.size();
  if (task_count == 0 || robot_names.empty())
    return candidate;

  if (candidate.preferred_robot_names_by_original_task.size() < task_count)
  {
    candidate.preferred_robot_names_by_original_task.resize(task_count);
  }

  std::vector<std::size_t> shuffled_indices(task_count);
  std::iota(shuffled_indices.begin(), shuffled_indices.end(), 0);
  std::shuffle(shuffled_indices.begin(), shuffled_indices.end(), rng);

  std::uniform_int_distribution<int> num_changes_dist(
      1, std::max<int>(1, static_cast<int>(task_count / 3)));
  int num_changes = num_changes_dist(rng);

  for (int i = 0; i < num_changes; ++i)
  {
    std::size_t task_idx = shuffled_indices[static_cast<std::size_t>(i)];
    const std::string &current =
        candidate.preferred_robot_names_by_original_task[task_idx];

    std::vector<std::string> alternatives;
    alternatives.reserve(robot_names.size());
    for (const auto &robot_name : robot_names)
    {
      if (robot_name != current)
        alternatives.push_back(robot_name);
    }

    if (alternatives.empty())
      continue;

    std::uniform_int_distribution<std::size_t> pick_dist(0, alternatives.size() - 1);
    candidate.preferred_robot_names_by_original_task[task_idx] =
        alternatives[pick_dist(rng)];
  }

  return candidate;
}

AllocationScenarioPlan mutate_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  std::size_t task_count = candidate.task_order.size();
  if (task_count < 2)
    return candidate;

  std::uniform_int_distribution<int> num_edits_dist(
      1, std::min<int>(3, static_cast<int>(task_count - 1)));
  int num_edits = num_edits_dist(rng);

  for (int edit = 0; edit < num_edits; ++edit)
  {
    std::uniform_int_distribution<std::size_t> index_dist(0, task_count - 1);
    std::size_t i = index_dist(rng);
    std::size_t j = index_dist(rng);
    while (j == i)
      j = index_dist(rng);

    std::bernoulli_distribution swap_or_relocate(0.5);
    if (swap_or_relocate(rng))
    {
      std::swap(candidate.task_order[i], candidate.task_order[j]);
      continue;
    }

    std::size_t moved_task = candidate.task_order[i];
    candidate.task_order.erase(candidate.task_order.begin() + static_cast<std::ptrdiff_t>(i));
    if (j > i)
      --j;
    candidate.task_order.insert(candidate.task_order.begin() + static_cast<std::ptrdiff_t>(j),
                                moved_task);
  }

  candidate.preferred_robot_names_by_original_task.clear();
  return candidate;
}

AllocationScenarioPlan mutate_full_shuffle_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  if (candidate.task_order.size() < 2)
    return candidate;

  std::shuffle(candidate.task_order.begin(), candidate.task_order.end(), rng);
  candidate.preferred_robot_names_by_original_task.clear();

  return candidate;
}

bool is_better_run(const AllocationRunSummary &candidate,
                   const AllocationRunSummary &current_best)
{
  if (!candidate.all_tasks_succeeded)
    return false;
  if (!current_best.all_tasks_succeeded)
    return true;
  return candidate.makespan + 1e-6 < current_best.makespan;
}

bool is_preferred_search_result(const AllocationRunSummary &candidate,
                                const AllocationRunSummary &current_best)
{
  if (candidate.all_tasks_succeeded != current_best.all_tasks_succeeded)
    return candidate.all_tasks_succeeded;

  if (candidate.all_tasks_succeeded)
    return candidate.makespan + 1e-6 < current_best.makespan;

  if (candidate.successful_tasks != current_best.successful_tasks)
    return candidate.successful_tasks > current_best.successful_tasks;

  if (candidate.failed_tasks != current_best.failed_tasks)
    return candidate.failed_tasks < current_best.failed_tasks;

  return false;
}

bool plans_have_same_order(const AllocationScenarioPlan &a,
                           const AllocationScenarioPlan &b)
{
  return a.task_order == b.task_order;
}

std::string task_order_signature(const AllocationScenarioPlan &plan)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < plan.task_order.size(); ++i)
  {
    if (i > 0)
      oss << "-";
    oss << plan.task_order[i];
  }
  return oss.str();
}

int count_assignment_preference_changes(const AllocationScenarioPlan &a,
                                        const AllocationScenarioPlan &b)
{
  std::size_t limit = std::max(a.preferred_robot_names_by_original_task.size(),
                               b.preferred_robot_names_by_original_task.size());
  int changes = 0;
  for (std::size_t i = 0; i < limit; ++i)
  {
    std::string left =
        (i < a.preferred_robot_names_by_original_task.size())
            ? a.preferred_robot_names_by_original_task[i]
            : "";
    std::string right =
        (i < b.preferred_robot_names_by_original_task.size())
            ? b.preferred_robot_names_by_original_task[i]
            : "";
    if (left != right)
      ++changes;
  }
  return changes;
}

int count_task_position_changes(const AllocationScenarioPlan &a,
                                const AllocationScenarioPlan &b)
{
  std::size_t limit = std::min(a.task_order.size(), b.task_order.size());
  int changes = 0;
  for (std::size_t i = 0; i < limit; ++i)
  {
    if (a.task_order[i] != b.task_order[i])
      ++changes;
  }

  changes += static_cast<int>(
      std::max(a.task_order.size(), b.task_order.size()) - limit);
  return changes;
}

bool same_execution_signature(const AllocationRunSummary &a,
                              const AllocationRunSummary &b)
{
  if (a.task_rows.size() != b.task_rows.size())
    return false;

  for (std::size_t i = 0; i < a.task_rows.size(); ++i)
  {
    const auto &lhs = a.task_rows[i];
    const auto &rhs = b.task_rows[i];
    if (lhs.object_name != rhs.object_name ||
        lhs.robot_name != rhs.robot_name ||
        lhs.status != rhs.status)
    {
      return false;
    }
  }

  return true;
}

std::string preview_task_order(const AllocationScenarioPlan &plan,
                               const std::vector<FinalAllocation> &loaded_sequence,
                               std::size_t max_items = 6)
{
  std::ostringstream oss;
  std::size_t shown = 0;
  for (std::size_t idx : plan.task_order)
  {
    if (idx >= loaded_sequence.size())
      continue;
    if (shown > 0)
      oss << ",";
    oss << loaded_sequence[idx].object.name;
    ++shown;
    if (shown >= max_items)
      break;
  }

  if (plan.task_order.size() > shown)
    oss << ",...";
  return oss.str();
}

std::string preview_assignment_changes(const AllocationScenarioPlan &base_plan,
                                       const AllocationScenarioPlan &candidate_plan,
                                       const std::vector<FinalAllocation> &loaded_sequence,
                                       std::size_t max_items = 4)
{
  std::ostringstream oss;
  std::size_t shown = 0;
  std::size_t limit = std::min(loaded_sequence.size(),
                               std::max(base_plan.preferred_robot_names_by_original_task.size(),
                                        candidate_plan.preferred_robot_names_by_original_task.size()));

  for (std::size_t i = 0; i < limit; ++i)
  {
    std::string before =
        (i < base_plan.preferred_robot_names_by_original_task.size())
            ? base_plan.preferred_robot_names_by_original_task[i]
            : "";
    std::string after =
        (i < candidate_plan.preferred_robot_names_by_original_task.size())
            ? candidate_plan.preferred_robot_names_by_original_task[i]
            : "";
    if (before == after)
      continue;

    if (shown > 0)
      oss << ", ";
    oss << loaded_sequence[i].object.name << ":" << before << "->" << after;
    ++shown;
    if (shown >= max_items)
      break;
  }

  if (shown == 0)
    return "none";

  if (count_assignment_preference_changes(base_plan, candidate_plan) >
      static_cast<int>(shown))
  {
    oss << ", ...";
  }

  return oss.str();
}

AllocationScenarioPlan make_distinct_assignment_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::string> &robot_names,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  for (int attempt = 0; attempt < 8; ++attempt)
  {
    candidate = mutate_assignment_plan(base_plan, robot_names, rng);
    if (count_assignment_preference_changes(base_plan, candidate) > 0)
      return candidate;
  }
  return candidate;
}

AllocationScenarioPlan make_distinct_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  for (int attempt = 0; attempt < 8; ++attempt)
  {
    candidate = mutate_sequence_plan(base_plan, rng);
    if (!plans_have_same_order(base_plan, candidate))
      return candidate;
  }
  return candidate;
}

std::vector<int> build_task_position_lookup(
    const AllocationScenarioPlan &plan,
    std::size_t task_count);

LearnedOrderConstraints make_learned_order_constraints(std::size_t task_count)
{
  LearnedOrderConstraints constraints;
  constraints.evidence_counts.assign(
      task_count, std::vector<int>(task_count, 0));
  constraints.enforced.assign(
      task_count, std::vector<bool>(task_count, false));
  return constraints;
}

bool order_constraint_path_exists(
    const LearnedOrderConstraints &constraints,
    std::size_t start_task,
    std::size_t goal_task)
{
  if (start_task >= constraints.enforced.size() ||
      goal_task >= constraints.enforced.size())
  {
    return false;
  }

  std::vector<bool> visited(constraints.enforced.size(), false);
  std::vector<std::size_t> stack = {start_task};
  visited[start_task] = true;

  while (!stack.empty())
  {
    std::size_t current = stack.back();
    stack.pop_back();
    if (current == goal_task)
      return true;

    for (std::size_t next = 0; next < constraints.enforced[current].size(); ++next)
    {
      if (!constraints.enforced[current][next] || visited[next])
        continue;
      visited[next] = true;
      stack.push_back(next);
    }
  }

  return false;
}

AllocationScenarioPlan project_plan_to_order_constraints(
    const AllocationScenarioPlan &plan,
    std::size_t task_count,
    const LearnedOrderConstraints &constraints)
{
  AllocationScenarioPlan projected = plan;
  auto base_order = normalized_task_order(plan, task_count);

  if (task_count == 0 || constraints.enforced.size() != task_count)
  {
    projected.task_order = std::move(base_order);
    return projected;
  }

  std::vector<int> preferred_rank(task_count, static_cast<int>(task_count));
  for (std::size_t i = 0; i < base_order.size(); ++i)
  {
    preferred_rank[base_order[i]] = static_cast<int>(i);
  }

  std::vector<int> indegree(task_count, 0);
  for (std::size_t before = 0; before < task_count; ++before)
  {
    for (std::size_t after = 0; after < task_count; ++after)
    {
      if (constraints.enforced[before][after])
        ++indegree[after];
    }
  }

  std::vector<bool> scheduled(task_count, false);
  projected.task_order.clear();
  projected.task_order.reserve(task_count);

  for (std::size_t step = 0; step < task_count; ++step)
  {
    std::size_t chosen = task_count;
    int chosen_rank = std::numeric_limits<int>::max();
    for (std::size_t task_idx = 0; task_idx < task_count; ++task_idx)
    {
      if (scheduled[task_idx] || indegree[task_idx] != 0)
        continue;

      int rank = preferred_rank[task_idx];
      if (rank < chosen_rank ||
          (rank == chosen_rank && task_idx < chosen))
      {
        chosen = task_idx;
        chosen_rank = rank;
      }
    }

    if (chosen >= task_count)
    {
      projected.task_order = std::move(base_order);
      return projected;
    }

    scheduled[chosen] = true;
    projected.task_order.push_back(chosen);
    for (std::size_t after = 0; after < task_count; ++after)
    {
      if (constraints.enforced[chosen][after])
        --indegree[after];
    }
  }

  return projected;
}

bool failure_reason_mentions_object(
    const std::string &failure_reason,
    const std::string &object_name)
{
  if (failure_reason.empty() || object_name.empty())
    return false;

  std::size_t pos = failure_reason.find(object_name);
  while (pos != std::string::npos)
  {
    bool left_ok = (pos == 0) ||
                   !std::isalnum(static_cast<unsigned char>(failure_reason[pos - 1]));
    std::size_t after_pos = pos + object_name.size();
    bool right_ok = (after_pos >= failure_reason.size()) ||
                    !std::isalnum(static_cast<unsigned char>(failure_reason[after_pos]));
    if (left_ok && right_ok)
      return true;
    pos = failure_reason.find(object_name, pos + 1);
  }

  return false;
}

bool note_order_constraint_evidence(
    LearnedOrderConstraints &constraints,
    std::size_t before_task,
    std::size_t after_task,
    int required_support,
    const std::vector<FinalAllocation> &loaded_sequence,
    const char *reason_tag)
{
  if (before_task >= constraints.evidence_counts.size() ||
      after_task >= constraints.evidence_counts.size() ||
      before_task == after_task)
  {
    return false;
  }

  int &support = constraints.evidence_counts[before_task][after_task];
  ++support;
  bool enforced_now = false;

  if (!constraints.enforced[before_task][after_task] &&
      support >= required_support &&
      !order_constraint_path_exists(constraints, after_task, before_task))
  {
    constraints.enforced[before_task][after_task] = true;
    ++constraints.enforced_count;
    enforced_now = true;
  }

  if (support == 1 || enforced_now)
  {
    std::cout << "[Learn][Order] "
              << loaded_sequence[before_task].object.name << " -> "
              << loaded_sequence[after_task].object.name
              << " support=" << support << "/" << required_support;
    if (reason_tag && *reason_tag)
      std::cout << " reason=" << reason_tag;
    if (enforced_now)
      std::cout << " enforced";
    std::cout << std::endl;
  }
  return enforced_now;
}

void learn_order_constraints_from_failed_summary(
    const AllocationRunSummary &failed_summary,
    const AllocationScenarioPlan &reference_plan,
    const std::vector<FinalAllocation> &loaded_sequence,
    LearnedOrderConstraints &constraints)
{
  if (failed_summary.all_tasks_succeeded || failed_summary.task_rows.empty() ||
      loaded_sequence.empty())
  {
    return;
  }

  auto candidate_order = normalized_task_order(
      failed_summary.plan, loaded_sequence.size());
  int failed_position = -1;
  const TaskCsvRow *failed_row = nullptr;
  for (std::size_t i = 0; i < failed_summary.task_rows.size(); ++i)
  {
    if (failed_summary.task_rows[i].status != "SUCCESS")
    {
      failed_position = static_cast<int>(i);
      failed_row = &failed_summary.task_rows[i];
      break;
    }
  }

  if (!failed_row || failed_position < 0 ||
      static_cast<std::size_t>(failed_position) >= candidate_order.size())
  {
    return;
  }

  std::size_t failed_task = candidate_order[static_cast<std::size_t>(failed_position)];
  auto candidate_positions = build_task_position_lookup(
      failed_summary.plan, loaded_sequence.size());
  auto reference_positions = build_task_position_lookup(
      reference_plan, loaded_sequence.size());

  std::vector<bool> handled_before(loaded_sequence.size(), false);

  for (std::size_t before_task = 0; before_task < loaded_sequence.size(); ++before_task)
  {
    if (before_task == failed_task ||
        candidate_positions[before_task] < 0 ||
        candidate_positions[failed_task] < 0)
    {
      continue;
    }

    if (!failure_reason_mentions_object(
            failed_row->failure_reason,
            loaded_sequence[before_task].object.name))
    {
      continue;
    }

    if (reference_positions[before_task] < reference_positions[failed_task] &&
        candidate_positions[before_task] > candidate_positions[failed_task])
    {
      note_order_constraint_evidence(
          constraints, before_task, failed_task, 1,
          loaded_sequence, "blocking-object");
      handled_before[before_task] = true;
    }
    else if (reference_positions[before_task] > reference_positions[failed_task] &&
             candidate_positions[before_task] < candidate_positions[failed_task])
    {
      note_order_constraint_evidence(
          constraints, failed_task, before_task, 1,
          loaded_sequence, "blocking-object");
      handled_before[before_task] = true;
    }
  }

  std::vector<std::size_t> missing_predecessors;
  std::vector<std::size_t> advanced_successors;
  for (std::size_t before_task = 0; before_task < loaded_sequence.size(); ++before_task)
  {
    if (before_task == failed_task || handled_before[before_task] ||
        reference_positions[before_task] < 0 ||
        reference_positions[failed_task] < 0 ||
        candidate_positions[before_task] < 0 ||
        candidate_positions[failed_task] < 0)
    {
      continue;
    }

    if (reference_positions[before_task] < reference_positions[failed_task] &&
        candidate_positions[before_task] > candidate_positions[failed_task])
    {
      missing_predecessors.push_back(before_task);
    }
    else if (reference_positions[before_task] > reference_positions[failed_task] &&
             candidate_positions[before_task] < candidate_positions[failed_task])
    {
      advanced_successors.push_back(before_task);
    }
  }

  std::sort(missing_predecessors.begin(), missing_predecessors.end(),
            [&](std::size_t lhs, std::size_t rhs)
            {
              return reference_positions[lhs] > reference_positions[rhs];
            });
  std::sort(advanced_successors.begin(), advanced_successors.end(),
            [&](std::size_t lhs, std::size_t rhs)
            {
              return reference_positions[lhs] < reference_positions[rhs];
            });

  int learned_from_reference = 0;
  for (std::size_t before_task : missing_predecessors)
  {
    int gap = reference_positions[failed_task] - reference_positions[before_task];
    int required_support = (gap <= 1) ? 1 : 2;
    note_order_constraint_evidence(
        constraints, before_task, failed_task, required_support,
        loaded_sequence, "failed-order");
    ++learned_from_reference;
    if (learned_from_reference >= 2)
      break;
  }

  int learned_successor_constraints = 0;
  for (std::size_t after_task : advanced_successors)
  {
    int gap = reference_positions[after_task] - reference_positions[failed_task];
    int required_support = (gap <= 1) ? 1 : 2;
    note_order_constraint_evidence(
        constraints, failed_task, after_task, required_support,
        loaded_sequence, "failed-order");
    ++learned_successor_constraints;
    if (learned_successor_constraints >= 2)
      break;
  }
}

bool sample_unique_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    AllocationScenarioPlan (*mutator)(const AllocationScenarioPlan &, std::mt19937 &),
    std::mt19937 &rng,
    std::unordered_set<std::string> &tried_signatures,
    const LearnedOrderConstraints *constraints,
    AllocationScenarioPlan *out_plan,
    int max_attempts = 256)
{
  if (!out_plan || !mutator)
    return false;

  for (int attempt = 0; attempt < max_attempts; ++attempt)
  {
    AllocationScenarioPlan candidate = mutator(base_plan, rng);
    if (constraints)
    {
      candidate = project_plan_to_order_constraints(
          candidate, base_plan.task_order.size(), *constraints);
    }
    if (plans_have_same_order(base_plan, candidate))
      continue;

    std::string signature = task_order_signature(candidate);
    if (!tried_signatures.insert(signature).second)
      continue;

    *out_plan = std::move(candidate);
    return true;
  }

  return false;
}

std::unordered_map<std::string, std::size_t> build_object_index_lookup(
    const std::vector<FinalAllocation> &loaded_sequence)
{
  std::unordered_map<std::string, std::size_t> lookup;
  lookup.reserve(loaded_sequence.size());
  for (std::size_t i = 0; i < loaded_sequence.size(); ++i)
  {
    lookup[loaded_sequence[i].object.name] = i;
  }
  return lookup;
}

std::vector<int> build_task_position_lookup(
    const AllocationScenarioPlan &plan,
    std::size_t task_count)
{
  auto order = normalized_task_order(plan, task_count);
  std::vector<int> positions(task_count, -1);
  for (std::size_t i = 0; i < order.size(); ++i)
  {
    positions[order[i]] = static_cast<int>(i);
  }
  return positions;
}

double summary_effective_completion_time(const AllocationRunSummary &summary)
{
  if (std::isfinite(summary.makespan))
    return summary.makespan;

  double completion = 0.0;
  for (const auto &row : summary.task_rows)
  {
    completion = std::max(completion, std::max(row.end_time, row.start_time));
  }
  return completion;
}

std::vector<double> compute_task_destroy_scores(
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence)
{
  auto object_lookup = build_object_index_lookup(loaded_sequence);
  const double ref_completion =
      std::max(1.0, summary_effective_completion_time(summary));

  std::vector<double> scores(loaded_sequence.size(), 1.0);
  for (const auto &row : summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it == object_lookup.end())
      continue;

    std::size_t task_idx = it->second;
    double end_time = std::max(row.end_time, row.start_time);
    double wait_component = std::sqrt(std::max(0.0, row.total_waiting));
    double end_ratio = end_time > 0.0 ? (end_time / ref_completion) : 0.0;
    double critical_component = 6.0 * std::max(0.0, end_ratio - 0.55);
    double tail_bonus = (end_ratio >= 0.85) ? 6.0 : 0.0;
    double failure_bonus = (row.status == "SUCCESS") ? 0.0 : 20.0;
    scores[task_idx] += 4.0 * wait_component + critical_component +
                        tail_bonus + failure_bonus;
  }

  return scores;
}

std::vector<std::size_t> sample_weighted_task_indices(
    const std::vector<double> &weights,
    std::size_t desired_count,
    std::mt19937 &rng)
{
  desired_count = std::min(desired_count, weights.size());
  std::vector<double> mutable_weights = weights;
  std::vector<std::size_t> selected;
  selected.reserve(desired_count);

  while (selected.size() < desired_count)
  {
    double weight_sum = 0.0;
    for (double weight : mutable_weights)
      weight_sum += std::max(0.0, weight);

    if (weight_sum <= 1e-9)
      break;

    std::discrete_distribution<std::size_t> pick_dist(
        mutable_weights.begin(), mutable_weights.end());
    std::size_t picked = pick_dist(rng);
    if (picked >= mutable_weights.size() || mutable_weights[picked] <= 0.0)
      continue;

    selected.push_back(picked);
    mutable_weights[picked] = 0.0;
  }

  if (selected.size() < desired_count)
  {
    std::vector<std::size_t> fallback;
    fallback.reserve(weights.size());
    for (std::size_t i = 0; i < weights.size(); ++i)
    {
      if (std::find(selected.begin(), selected.end(), i) == selected.end())
        fallback.push_back(i);
    }
    std::shuffle(fallback.begin(), fallback.end(), rng);
    while (selected.size() < desired_count && !fallback.empty())
    {
      selected.push_back(fallback.back());
      fallback.pop_back();
    }
  }

  return selected;
}

std::vector<std::size_t> destroy_random_tasks(
    std::size_t task_count,
    std::size_t destroy_count,
    std::mt19937 &rng)
{
  std::vector<std::size_t> tasks(task_count);
  std::iota(tasks.begin(), tasks.end(), 0);
  std::shuffle(tasks.begin(), tasks.end(), rng);
  tasks.resize(std::min(destroy_count, tasks.size()));
  return tasks;
}

std::vector<std::size_t> destroy_wait_neighborhood_tasks(
    const AllocationScenarioPlan &plan,
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence,
    std::size_t destroy_count,
    std::mt19937 &rng)
{
  destroy_count = std::min(destroy_count, loaded_sequence.size());
  auto scores = compute_task_destroy_scores(summary, loaded_sequence);
  auto order = normalized_task_order(plan, loaded_sequence.size());
  auto positions = build_task_position_lookup(plan, loaded_sequence.size());
  auto object_lookup = build_object_index_lookup(loaded_sequence);

  std::vector<std::string> robot_by_task(loaded_sequence.size(), "");
  for (const auto &row : summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it != object_lookup.end())
      robot_by_task[it->second] = row.robot_name;
  }

  std::vector<bool> selected(loaded_sequence.size(), false);
  std::vector<std::size_t> destroyed;
  destroyed.reserve(destroy_count);

  auto add_task = [&](std::size_t task_idx)
  {
    if (task_idx >= selected.size() || selected[task_idx])
      return;
    selected[task_idx] = true;
    destroyed.push_back(task_idx);
  };

  std::size_t anchor_count = std::max<std::size_t>(1, (destroy_count + 1) / 2);
  auto anchors = sample_weighted_task_indices(scores, anchor_count, rng);
  for (std::size_t anchor : anchors)
  {
    add_task(anchor);
    if (destroyed.size() >= destroy_count)
      break;

    int position = positions[anchor];
    if (position >= 0)
    {
      if (position > 0)
        add_task(order[static_cast<std::size_t>(position - 1)]);
      if (destroyed.size() >= destroy_count)
        break;
      if (static_cast<std::size_t>(position + 1) < order.size())
        add_task(order[static_cast<std::size_t>(position + 1)]);
      if (destroyed.size() >= destroy_count)
        break;
    }

    const std::string &robot_name = robot_by_task[anchor];
    if (!robot_name.empty())
    {
      std::size_t best_peer = loaded_sequence.size();
      double best_score = -1.0;
      for (std::size_t task_idx = 0; task_idx < robot_by_task.size(); ++task_idx)
      {
        if (selected[task_idx] || robot_by_task[task_idx] != robot_name)
          continue;
        if (scores[task_idx] > best_score)
        {
          best_score = scores[task_idx];
          best_peer = task_idx;
        }
      }
      if (best_peer < loaded_sequence.size())
        add_task(best_peer);
    }

    if (destroyed.size() >= destroy_count)
      break;
  }

  auto extras = sample_weighted_task_indices(scores, destroy_count, rng);
  for (std::size_t task_idx : extras)
  {
    add_task(task_idx);
    if (destroyed.size() >= destroy_count)
      break;
  }

  return destroyed;
}

std::vector<std::size_t> destroy_critical_suffix_tasks(
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence,
    std::size_t destroy_count,
    std::mt19937 &rng)
{
  destroy_count = std::min(destroy_count, loaded_sequence.size());
  auto object_lookup = build_object_index_lookup(loaded_sequence);
  auto scores = compute_task_destroy_scores(summary, loaded_sequence);

  struct TimedTask
  {
    std::size_t task_idx = 0;
    std::string robot_name;
    double end_time = -1.0;
  };

  std::vector<TimedTask> timed_tasks;
  timed_tasks.reserve(summary.task_rows.size());
  for (const auto &row : summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it == object_lookup.end())
      continue;
    timed_tasks.push_back({it->second, row.robot_name,
                           std::max(row.end_time, row.start_time)});
  }

  std::sort(timed_tasks.begin(), timed_tasks.end(),
            [](const TimedTask &a, const TimedTask &b)
            { return a.end_time > b.end_time; });

  std::vector<bool> selected(loaded_sequence.size(), false);
  std::vector<std::size_t> destroyed;
  destroyed.reserve(destroy_count);

  auto add_task = [&](std::size_t task_idx)
  {
    if (task_idx >= selected.size() || selected[task_idx])
      return;
    selected[task_idx] = true;
    destroyed.push_back(task_idx);
  };

  std::string critical_robot =
      timed_tasks.empty() ? "" : timed_tasks.front().robot_name;
  for (const auto &task : timed_tasks)
  {
    if (!critical_robot.empty() && task.robot_name == critical_robot)
      add_task(task.task_idx);
    if (destroyed.size() >= destroy_count)
      return destroyed;
  }

  for (const auto &task : timed_tasks)
  {
    add_task(task.task_idx);
    if (destroyed.size() >= destroy_count)
      return destroyed;
  }

  auto extras = sample_weighted_task_indices(scores, destroy_count, rng);
  for (std::size_t task_idx : extras)
  {
    add_task(task_idx);
    if (destroyed.size() >= destroy_count)
      break;
  }

  return destroyed;
}

AllocationScenarioPlan remove_tasks_from_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::size_t> &removed_tasks,
    std::size_t task_count)
{
  AllocationScenarioPlan partial = base_plan;
  partial.task_order = normalized_task_order(base_plan, task_count);
  partial.preferred_robot_names_by_original_task.resize(task_count);

  std::vector<bool> removed(task_count, false);
  for (std::size_t task_idx : removed_tasks)
  {
    if (task_idx < task_count)
    {
      removed[task_idx] = true;
      partial.preferred_robot_names_by_original_task[task_idx].clear();
    }
  }

  partial.task_order.erase(
      std::remove_if(partial.task_order.begin(), partial.task_order.end(),
                     [&](std::size_t task_idx)
                     { return task_idx < removed.size() && removed[task_idx]; }),
      partial.task_order.end());
  return partial;
}

AllocationScenarioPlan complete_partial_plan_with_pending(
    const AllocationScenarioPlan &partial_plan,
    const std::vector<std::size_t> &pending_tasks,
    const std::vector<int> &base_positions,
    const AllocationScenarioPlan &base_plan,
    std::size_t task_count)
{
  AllocationScenarioPlan completed = partial_plan;
  completed.preferred_robot_names_by_original_task.resize(task_count);

  std::vector<std::size_t> pending_sorted = pending_tasks;
  std::sort(pending_sorted.begin(), pending_sorted.end(),
            [&](std::size_t a, std::size_t b)
            {
              int pos_a = (a < base_positions.size()) ? base_positions[a] : -1;
              int pos_b = (b < base_positions.size()) ? base_positions[b] : -1;
              return pos_a < pos_b;
            });

  std::size_t base_pref_count =
      base_plan.preferred_robot_names_by_original_task.size();
  for (std::size_t task_idx : pending_sorted)
  {
    std::size_t insert_pos = completed.task_order.size();
    if (task_idx < base_positions.size() && base_positions[task_idx] >= 0)
    {
      insert_pos = std::min<std::size_t>(
          static_cast<std::size_t>(base_positions[task_idx]),
          completed.task_order.size());
    }
    completed.task_order.insert(
        completed.task_order.begin() + static_cast<std::ptrdiff_t>(insert_pos),
        task_idx);

    if (task_idx < base_pref_count &&
        completed.preferred_robot_names_by_original_task[task_idx].empty())
    {
      completed.preferred_robot_names_by_original_task[task_idx] =
          base_plan.preferred_robot_names_by_original_task[task_idx];
    }
  }

  return completed;
}

std::vector<std::size_t> build_sampled_insertion_positions(
    const AllocationScenarioPlan &partial_plan,
    int original_position,
    std::mt19937 &rng,
    std::size_t max_positions = 4)
{
  std::size_t total_positions = partial_plan.task_order.size() + 1;
  if (total_positions <= max_positions)
  {
    std::vector<std::size_t> positions(total_positions);
    std::iota(positions.begin(), positions.end(), 0);
    return positions;
  }

  std::vector<std::size_t> positions;
  positions.reserve(max_positions);
  auto add_position = [&](std::size_t position)
  {
    position = std::min(position, total_positions - 1);
    if (std::find(positions.begin(), positions.end(), position) == positions.end())
      positions.push_back(position);
  };

  add_position(0);
  add_position(total_positions - 1);

  std::size_t clamped_original = 0;
  if (original_position > 0)
  {
    clamped_original = std::min<std::size_t>(
        static_cast<std::size_t>(original_position), total_positions - 1);
  }
  add_position(clamped_original);
  if (clamped_original > 0)
    add_position(clamped_original - 1);
  if (clamped_original + 1 < total_positions)
    add_position(clamped_original + 1);

  std::uniform_int_distribution<std::size_t> pos_dist(0, total_positions - 1);
  while (positions.size() < max_positions)
  {
    add_position(pos_dist(rng));
  }

  std::sort(positions.begin(), positions.end());
  return positions;
}

AllocationRunSummary repair_destroyed_tasks_with_sampled_insertion(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<std::size_t> &destroyed_tasks,
    const std::vector<std::string> &robot_names,
    const std::vector<double> &destroy_scores,
    std::uint32_t parking_seed,
    std::uint32_t repair_seed,
    const std::string &label,
    bool disable_local_silencer)
{
  std::mt19937 rng(repair_seed);
  RuntimeOptions lns_options = options;
  if (!lns_options.enable_lns_fine_segment_retry)
    lns_options.enable_fine_segment_retry = false;

  std::size_t task_count = loaded_sequence.size();
  AllocationScenarioPlan partial_plan =
      remove_tasks_from_plan(base_plan, destroyed_tasks, task_count);
  auto base_positions = build_task_position_lookup(base_plan, task_count);
  auto object_lookup = build_object_index_lookup(loaded_sequence);

  std::vector<std::string> executed_robot_by_task(task_count, "");
  std::vector<double> wait_by_task(task_count, 0.0);
  for (const auto &row : base_summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it != object_lookup.end())
    {
      executed_robot_by_task[it->second] = row.robot_name;
      wait_by_task[it->second] = row.total_waiting;
    }
  }

  std::vector<std::size_t> repair_order = destroyed_tasks;
  std::shuffle(repair_order.begin(), repair_order.end(), rng);
  std::stable_sort(repair_order.begin(), repair_order.end(),
                   [&](std::size_t a, std::size_t b)
                   {
                     double score_a = (a < destroy_scores.size()) ? destroy_scores[a] : 0.0;
                     double score_b = (b < destroy_scores.size()) ? destroy_scores[b] : 0.0;
                     return score_a > score_b;
                   });

  for (std::size_t task_idx : repair_order)
  {
    auto candidate_positions = build_sampled_insertion_positions(
        partial_plan,
        (task_idx < base_positions.size()) ? base_positions[task_idx] : -1,
        rng);

    std::size_t chosen_position = candidate_positions.empty() ? partial_plan.task_order.size()
                                                              : candidate_positions.front();
    int original_position =
        (task_idx < base_positions.size()) ? base_positions[task_idx] : -1;
    if (wait_by_task[task_idx] > 0.5)
    {
      for (std::size_t pos : candidate_positions)
      {
        if (original_position >= 0 &&
            pos <= static_cast<std::size_t>(original_position))
        {
          chosen_position = pos;
          break;
        }
      }
    }
    else if (!candidate_positions.empty())
    {
      std::uniform_int_distribution<std::size_t> pick_pos_dist(
          0, candidate_positions.size() - 1);
      chosen_position = candidate_positions[pick_pos_dist(rng)];
    }

    std::string chosen_preference = "";
    const std::string base_preference =
        (task_idx < base_plan.preferred_robot_names_by_original_task.size())
            ? base_plan.preferred_robot_names_by_original_task[task_idx]
            : "";
    const std::string executed_robot =
        (task_idx < executed_robot_by_task.size())
            ? executed_robot_by_task[task_idx]
            : "";

    if (wait_by_task[task_idx] > 1.0 && !executed_robot.empty())
    {
      std::vector<std::string> alternative_robots;
      for (const auto &robot_name : robot_names)
      {
        if (robot_name != executed_robot)
          alternative_robots.push_back(robot_name);
      }
      if (!alternative_robots.empty())
      {
        std::shuffle(alternative_robots.begin(), alternative_robots.end(), rng);
        chosen_preference = alternative_robots.front();
      }
    }

    if (chosen_preference.empty())
    {
      std::vector<std::string> fallback_preferences;
      auto add_preference = [&](const std::string &pref)
      {
        if (std::find(fallback_preferences.begin(), fallback_preferences.end(), pref) ==
            fallback_preferences.end())
        {
          fallback_preferences.push_back(pref);
        }
      };

      add_preference(base_preference);
      add_preference(executed_robot);
      add_preference("");

      std::vector<std::string> shuffled_robot_names = robot_names;
      std::shuffle(shuffled_robot_names.begin(), shuffled_robot_names.end(), rng);
      for (const auto &robot_name : shuffled_robot_names)
      {
        add_preference(robot_name);
        if (fallback_preferences.size() >= 3)
          break;
      }

      if (!fallback_preferences.empty())
      {
        std::uniform_int_distribution<std::size_t> pref_pick_dist(
            0, fallback_preferences.size() - 1);
        chosen_preference = fallback_preferences[pref_pick_dist(rng)];
      }
    }

    partial_plan.preferred_robot_names_by_original_task.resize(task_count);
    partial_plan.task_order.insert(
        partial_plan.task_order.begin() + static_cast<std::ptrdiff_t>(chosen_position),
        task_idx);
    partial_plan.preferred_robot_names_by_original_task[task_idx] =
        chosen_preference;
  }

  partial_plan = project_plan_to_order_constraints(
      partial_plan, task_count, constraints);

  auto repaired = execute_allocation_scenario(
      loaded_sequence, lns_options, partial_plan, label,
      parking_seed, disable_local_silencer);
  repaired.summary.label = label;

  for (int refine_iter = 0; refine_iter < 2 && !destroyed_tasks.empty(); ++refine_iter)
  {
    AllocationScenarioPlan candidate_plan = repaired.summary.plan;
    candidate_plan.preferred_robot_names_by_original_task.resize(task_count);

    std::uniform_int_distribution<std::size_t> destroyed_pick_dist(
        0, destroyed_tasks.size() - 1);
    std::size_t picked_task = destroyed_tasks[destroyed_pick_dist(rng)];

    std::bernoulli_distribution swap_or_reassign(0.5);
    if (swap_or_reassign(rng) && candidate_plan.task_order.size() > 1)
    {
      auto positions = build_task_position_lookup(candidate_plan, task_count);
      int current_pos = (picked_task < positions.size()) ? positions[picked_task] : -1;
      if (current_pos >= 0)
      {
        std::uniform_int_distribution<std::size_t> pos_dist(
            0, candidate_plan.task_order.size() - 1);
        std::size_t new_pos = pos_dist(rng);
        if (new_pos != static_cast<std::size_t>(current_pos))
        {
          candidate_plan.task_order.erase(
              candidate_plan.task_order.begin() + current_pos);
          if (new_pos > static_cast<std::size_t>(current_pos))
            --new_pos;
          candidate_plan.task_order.insert(
              candidate_plan.task_order.begin() + static_cast<std::ptrdiff_t>(new_pos),
              picked_task);
        }
      }
    }
    else
    {
      std::vector<std::string> preference_options;
      preference_options.push_back("");
      for (const auto &robot_name : robot_names)
      {
        if (std::find(preference_options.begin(), preference_options.end(), robot_name) ==
            preference_options.end())
        {
          preference_options.push_back(robot_name);
        }
      }
      std::uniform_int_distribution<std::size_t> pref_dist(
          0, preference_options.size() - 1);
      candidate_plan.preferred_robot_names_by_original_task[picked_task] =
          preference_options[pref_dist(rng)];
    }

    candidate_plan = project_plan_to_order_constraints(
        candidate_plan, task_count, constraints);

    auto refined = execute_allocation_scenario(
        loaded_sequence, lns_options, candidate_plan, label,
        parking_seed, disable_local_silencer);
    if (is_preferred_search_result(refined.summary, repaired.summary))
    {
      repaired = std::move(refined);
      repaired.summary.label = label;
    }
  }

  return repaired.summary;
}

void print_comparison_line(const AllocationRunSummary &summary,
                           double greedy_makespan)
{
  std::cout << "[Compare] " << summary.label << ": ";
  if (!summary.all_tasks_succeeded)
  {
    std::cout << "infeasible (" << summary.successful_tasks << "/"
              << (summary.successful_tasks + summary.failed_tasks)
              << " tasks succeeded)" << std::endl;
    return;
  }

  std::cout << "makespan=" << std::fixed << std::setprecision(2)
            << summary.makespan << "s";
  if (std::isfinite(greedy_makespan))
  {
    double delta = summary.makespan - greedy_makespan;
    std::cout << " (delta vs greedy="
              << (delta >= 0.0 ? "+" : "") << delta << "s)";
  }
  std::cout << ", seed=" << summary.parking_seed << std::endl;
}

std::vector<std::string> collect_robot_names(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options)
{
  ScopedStreamSilencer silencer(true);
  Params params;
  std::unordered_map<std::string, EntityMeta *> entities;
  TimeTable timetable(0.5);
  std::vector<RobotMeta *> all_robots;
  initialize_environment(loaded_sequence, options,
                         mix_seed(options.base_random_seed, 0x13579bdu),
                         params, entities, timetable, all_robots, false);

  std::vector<std::string> robot_names;
  robot_names.reserve(all_robots.size());
  for (auto *robot : all_robots)
  {
    if (robot)
      robot_names.push_back(robot->name);
  }

  cleanup_entities(entities);
  return robot_names;
}

AllocationRunSummary make_placeholder_summary(
    const std::string &label,
    const AllocationScenarioPlan &plan)
{
  AllocationRunSummary summary;
  summary.label = label;
  summary.plan = plan;
  summary.parking_seed = 0;
  summary.all_tasks_succeeded = false;
  summary.successful_tasks = 0;
  summary.failed_tasks = 0;
  summary.makespan = std::numeric_limits<double>::infinity();
  return summary;
}

std::string format_feasible_makespan_list(
    const std::vector<SearchTrialRecord> &records)
{
  std::ostringstream oss;
  bool first = true;
  for (const auto &record : records)
  {
    if (!record.feasible || !std::isfinite(record.makespan))
      continue;

    if (!first)
      oss << ", ";
    oss << std::fixed << std::setprecision(2) << record.makespan;
    first = false;
  }

  return first ? "none" : oss.str();
}

void print_search_method_summary(
    const std::string &label,
    bool has_feasible_candidate,
    const AllocationRunSummary &best_feasible,
    bool has_partial_candidate,
    const AllocationRunSummary &best_partial,
    double greedy_makespan)
{
  if (has_feasible_candidate)
  {
    print_comparison_line(best_feasible, greedy_makespan);
    return;
  }

  std::cout << "[Compare] " << label << ": ";
  if (has_partial_candidate)
  {
    std::cout << "no feasible candidate";
    if (best_partial.successful_tasks + best_partial.failed_tasks > 0)
    {
      std::cout << " (best partial " << best_partial.successful_tasks << "/"
                << (best_partial.successful_tasks + best_partial.failed_tasks)
                << " tasks, seed=" << best_partial.parking_seed << ")";
    }
    std::cout << std::endl;
    return;
  }

  std::cout << "no candidates evaluated" << std::endl;
}

int phastar_push_demo_main(int argc, char **argv)
{
  RuntimeOptions runtime_options = parse_runtime_options(argc, argv);
  ReloPush::HandoffInstanceInfo instance_info;
  std::vector<FinalAllocation> loadedSequence;
  std::unique_ptr<ReloPush::FinalSequenceHandoffServer> handoff_server;
  if (!load_data(runtime_options, instance_info, loadedSequence, handoff_server))
  {
    return 1;
  }
  const double relopush_single_robot_makespan =
      compute_relopush_single_robot_makespan(loadedSequence);

  auto notify_relopush_if_needed =
      [&](bool success, const std::string &detail)
  {
    if (!handoff_server || !handoff_server->hasPendingRequest())
    {
      return;
    }

    try
    {
      handoff_server->sendReply(ReloPush::makeMarsReply(success, detail));
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[Integration] Failed to send completion reply to ReloPush: "
                << ex.what() << std::endl;
    }
  };

  DEBUG_VIS = runtime_options.debug_vis;
  print_runtime_options(runtime_options);
  std::cout << "[Config] Instance file: " << instance_info.file_name << std::endl;
  std::cout << "[Config] Instance index: " << instance_info.instance_index << std::endl;
  std::cout << "[Config] ReloPush single-robot makespan: "
            << std::fixed << std::setprecision(2)
            << relopush_single_robot_makespan << "s" << std::endl;

  if (runtime_options.visualize_relopush_plan)
  {
    visualize_relopush_plan(argc, argv, loadedSequence, instance_info, runtime_options);
  }

  AllocationScenarioPlan greedy_plan = make_identity_plan(loadedSequence.size());
  const std::string greedy_label = "greedy";
  const std::string assignment_label = "fixed-sequence-random-reassign";
  const std::string lns_label = "lns-adaptive";
  const std::string sequence_label = "random-task-sequence";
  const std::uint32_t greedy_parking_seed =
      mix_seed(runtime_options.base_random_seed, 0xC0FFEE01u);
  const bool greedy_only_run =
      runtime_options.assignment_search_iterations <= 0 &&
      runtime_options.local_sequence_search_iterations <= 0 &&
      runtime_options.shuffle_sequence_search_iterations <= 0 &&
      runtime_options.lns_iterations <= 0;

  if (greedy_only_run)
  {
    std::cout << "[Search] No alternative search enabled. Running greedy scenario directly";
    if (runtime_options.enable_visualization)
      std::cout << " with visualization";
    else
      std::cout << " headless";
    std::cout << "." << std::endl;

    auto greedy_executed = execute_allocation_scenario(
        loadedSequence, runtime_options, greedy_plan, greedy_label,
        greedy_parking_seed, true);
    const auto &greedy_summary = greedy_executed.summary;

    std::cout << "[Compare] Makespan summary" << std::endl;
    print_comparison_line(greedy_summary, greedy_summary.makespan);

    std::string greedy_csv = std::string(CMAKE_SOURCE_DIR) +
                             "/task_execution_log_greedy.csv";
    write_task_csv_log(greedy_csv, greedy_summary.task_rows);

    std::string csv_path = std::string(CMAKE_SOURCE_DIR) + "/task_execution_log.csv";
    write_task_csv_log(csv_path, greedy_summary.task_rows);
    log_timetable_orientation_diagnostics(greedy_summary.label,
                                          greedy_executed.timetable,
                                          greedy_executed.entities,
                                          greedy_executed.params);

    std::string comparison_csv =
        std::string(CMAKE_SOURCE_DIR) + "/allocation_search_summary.csv";
    write_allocation_search_summary_csv(comparison_csv, {greedy_summary},
                                        greedy_summary.makespan);

    std::string instance_record_csv =
        std::string(CMAKE_SOURCE_DIR) + "/results/mars_instance_record_" +
        sanitize_filename_component(instance_info.file_name) + ".csv";
    write_instance_run_record_csv(
        instance_record_csv,
        instance_info,
        relopush_single_robot_makespan,
        greedy_summary.makespan,
        std::numeric_limits<double>::infinity(),
        runtime_options.lns_iterations,
        greedy_summary.label,
        greedy_summary.makespan);

    export_result_summary_figure(runtime_options, instance_info,
                                 greedy_summary.label,
                                 greedy_executed.timetable,
                                 greedy_executed.entities,
                                 greedy_executed.params);

    notify_relopush_if_needed(
        greedy_summary.all_tasks_succeeded,
        greedy_summary.all_tasks_succeeded ? "greedy-planning-complete"
                                           : "greedy-planning-failed");

    if (runtime_options.enable_visualization)
    {
      show_results(argc, argv, greedy_executed.timetable, greedy_executed.entities,
                   greedy_executed.params);
    }

    return greedy_summary.all_tasks_succeeded ? 0 : 1;
  }

  std::cout << "[Search] Evaluating greedy baseline..." << std::endl;
  AllocationRunSummary greedy_summary;
  {
    auto greedy_executed = execute_allocation_scenario(
        loadedSequence, runtime_options, greedy_plan, greedy_label,
        greedy_parking_seed, false);
    greedy_summary = greedy_executed.summary;
  }

  std::vector<std::string> robot_names = collect_robot_names(
      loadedSequence, runtime_options);
  std::mt19937 search_rng(
      mix_seed(runtime_options.base_random_seed, 0x51A7BEEFu));
  const int lns_batch_size = std::max(1, runtime_options.lns_threads);

  AllocationScenarioPlan greedy_assignment_plan =
      make_assignment_plan_from_greedy(greedy_summary);
  AllocationRunSummary assignment_best_feasible =
      make_placeholder_summary(assignment_label, greedy_assignment_plan);
  AllocationRunSummary assignment_best_partial =
      make_placeholder_summary(assignment_label, greedy_assignment_plan);
  AllocationRunSummary assignment_guiding_summary = greedy_summary;
  assignment_guiding_summary.label = assignment_label;
  assignment_guiding_summary.plan = greedy_assignment_plan;
  AllocationScenarioPlan assignment_current_plan = greedy_assignment_plan;
  bool assignment_has_partial_candidate = false;
  bool assignment_has_feasible_candidate = false;

  if (runtime_options.assignment_search_iterations > 0 && !robot_names.empty())
  {
    std::cout << "[Search] Randomized robot reassignment search (fixed task sequence)..."
              << std::endl;
    for (int iter = 0; iter < runtime_options.assignment_search_iterations;)
    {
      AllocationScenarioPlan batch_base_plan = assignment_current_plan;
      std::vector<AssignmentSearchCandidate> batch_candidates;
      std::vector<ScenarioEvaluationRequest> batch_requests;
      batch_candidates.reserve(static_cast<std::size_t>(lns_batch_size));
      batch_requests.reserve(static_cast<std::size_t>(lns_batch_size));

      int batch_end = std::min(runtime_options.assignment_search_iterations,
                               iter + lns_batch_size);
      for (; iter < batch_end; ++iter)
      {
        AssignmentSearchCandidate candidate;
        candidate.iteration = iter + 1;
        candidate.plan = make_distinct_assignment_plan(
            batch_base_plan, robot_names, search_rng);
        candidate.parking_seed = search_rng();
        candidate.preference_changes = count_assignment_preference_changes(
            batch_base_plan, candidate.plan);
        candidate.preview_changes = preview_assignment_changes(
            batch_base_plan, candidate.plan, loadedSequence);

        std::cout << "[Search][Assign " << candidate.iteration << "/"
                  << runtime_options.assignment_search_iterations << "] "
                  << "pref_changes=" << candidate.preference_changes
                  << " {" << candidate.preview_changes
                  << "}, seed=" << candidate.parking_seed << std::endl;

        batch_requests.push_back(
            {candidate.plan, assignment_label, candidate.parking_seed});
        batch_candidates.push_back(std::move(candidate));
      }

      auto batch_results = evaluate_scenario_batch(
          loadedSequence, runtime_options, batch_requests);

      for (std::size_t batch_idx = 0; batch_idx < batch_candidates.size(); ++batch_idx)
      {
        const auto &candidate_info = batch_candidates[batch_idx];
        auto candidate_summary = batch_results[batch_idx].summary;

        std::cout << "[Search][Assign " << candidate_info.iteration << "] ";
        if (candidate_summary.all_tasks_succeeded)
        {
          std::cout << "makespan=" << std::fixed << std::setprecision(2)
                    << candidate_summary.makespan << "s";
        }
        else
        {
          std::cout << "infeasible (" << candidate_summary.successful_tasks << "/"
                    << (candidate_summary.successful_tasks + candidate_summary.failed_tasks)
                    << " tasks)";
        }
        std::cout << ", execution="
                  << (same_execution_signature(candidate_summary, greedy_summary)
                          ? "same-as-greedy"
                          : "changed")
                  << std::endl;

        if (!assignment_has_partial_candidate ||
            is_preferred_search_result(candidate_summary, assignment_best_partial))
        {
          assignment_best_partial = candidate_summary;
          assignment_best_partial.label = assignment_label;
          assignment_has_partial_candidate = true;
          if (!candidate_summary.all_tasks_succeeded)
          {
            std::cout << "[Search][Assign " << candidate_info.iteration
                      << "] new best partial candidate recorded." << std::endl;
          }
        }

        if (candidate_summary.all_tasks_succeeded &&
            (!assignment_has_feasible_candidate ||
             is_better_run(candidate_summary, assignment_best_feasible)))
        {
          assignment_best_feasible = candidate_summary;
          assignment_best_feasible.label = assignment_label;
          assignment_has_feasible_candidate = true;
          std::cout << "[Search][Assign " << candidate_info.iteration
                    << "] new feasible-best candidate recorded." << std::endl;
        }

        if (is_preferred_search_result(candidate_summary, assignment_guiding_summary))
        {
          assignment_guiding_summary = candidate_summary;
          assignment_guiding_summary.label = assignment_label;
          assignment_current_plan = candidate_summary.plan;
        }
      }
    }
  }
  else
  {
    std::cout << "[Search] Skipping fixed-sequence reassignment search." << std::endl;
  }

  AllocationRunSummary assignment_report =
      assignment_has_feasible_candidate
          ? assignment_best_feasible
          : assignment_best_partial;

  AllocationScenarioPlan shuffled_sequence_plan = greedy_plan;
  shuffled_sequence_plan.preferred_robot_names_by_original_task.clear();
  std::vector<SearchTrialRecord> sequence_trial_records;
  std::unordered_set<std::string> tried_sequence_signatures;
  tried_sequence_signatures.insert(task_order_signature(greedy_plan));
  LearnedOrderConstraints disabled_order_constraints =
      make_learned_order_constraints(loadedSequence.size());
  int local_sequence_enforced_constraints = 0;
  int full_shuffle_enforced_constraints = 0;
  int lns_enforced_constraints = 0;
  auto run_sequence_method =
      [&](const std::string &label,
          const std::string &banner,
          const std::string &progress_tag,
          int iterations,
          AllocationScenarioPlan (*mutator)(const AllocationScenarioPlan &, std::mt19937 &),
          int *out_enforced_constraint_count)
  {
    SequenceSearchOutcome outcome;
    outcome.best_feasible = make_placeholder_summary(label, shuffled_sequence_plan);
    outcome.best_partial = make_placeholder_summary(label, shuffled_sequence_plan);
    LearnedOrderConstraints method_constraints =
        make_learned_order_constraints(loadedSequence.size());
    AllocationRunSummary method_order_learning_reference_summary = greedy_summary;
    auto maybe_update_method_order_learning_reference =
        [&](const AllocationRunSummary &summary)
    {
      if (summary.all_tasks_succeeded &&
          is_better_run(summary, method_order_learning_reference_summary))
      {
        method_order_learning_reference_summary = summary;
      }
    };

    if (iterations <= 0 || loadedSequence.size() <= 1)
    {
      if (out_enforced_constraint_count)
        *out_enforced_constraint_count = 0;
      std::cout << "[Search] Skipping " << banner << "." << std::endl;
      return outcome;
    }

    std::cout << "[Search] " << banner << "..." << std::endl;
    for (int iter = 0; iter < iterations;)
    {
      std::vector<SequenceSearchCandidate> batch_candidates;
      std::vector<ScenarioEvaluationRequest> batch_requests;
      batch_candidates.reserve(static_cast<std::size_t>(lns_batch_size));
      batch_requests.reserve(static_cast<std::size_t>(lns_batch_size));

      int batch_end = std::min(iterations, iter + lns_batch_size);
      for (; iter < batch_end; ++iter)
      {
        AllocationScenarioPlan candidate_plan;
        if (!sample_unique_sequence_plan(greedy_plan, mutator, search_rng,
                                         tried_sequence_signatures,
                                         runtime_options.enable_order_constraint_learning
                                             ? &method_constraints
                                             : nullptr,
                                         &candidate_plan))
        {
          std::cout << "[Search][" << progress_tag
                    << "] Unable to sample a new unseen order after repeated attempts. "
                    << "Stopping early at iteration " << (iter + 1) << "."
                    << std::endl;
          iter = iterations;
          break;
        }

        SequenceSearchCandidate candidate;
        candidate.iteration = iter + 1;
        candidate.plan = candidate_plan;
        candidate.parking_seed = search_rng();
        candidate.position_changes = count_task_position_changes(
            greedy_plan, candidate.plan);
        candidate.preview_order = preview_task_order(candidate.plan, loadedSequence);

        std::cout << "[Search][" << progress_tag << " " << candidate.iteration << "/"
                  << iterations << "] "
                  << "position_changes=" << candidate.position_changes
                  << ", order={" << candidate.preview_order
                  << "}, seed=" << candidate.parking_seed << std::endl;

        batch_requests.push_back(
            {candidate.plan, label, candidate.parking_seed});
        batch_candidates.push_back(std::move(candidate));
      }

      auto batch_results = evaluate_scenario_batch(
          loadedSequence, runtime_options, batch_requests);

      for (std::size_t batch_idx = 0; batch_idx < batch_candidates.size(); ++batch_idx)
      {
        const auto &candidate_info = batch_candidates[batch_idx];
        auto candidate_summary = batch_results[batch_idx].summary;

        std::cout << "[Search][" << progress_tag << " " << candidate_info.iteration
                  << "] ";
        if (candidate_summary.all_tasks_succeeded)
        {
          std::cout << "makespan=" << std::fixed << std::setprecision(2)
                    << candidate_summary.makespan << "s";
        }
        else
        {
          std::cout << "infeasible (" << candidate_summary.successful_tasks << "/"
                    << (candidate_summary.successful_tasks + candidate_summary.failed_tasks)
                    << " tasks)";
        }
        std::cout << ", execution="
                  << (same_execution_signature(candidate_summary, greedy_summary)
                          ? "same-as-greedy"
                          : "changed")
                  << std::endl;

        SearchTrialRecord record;
        record.search_label = label;
        record.iteration = candidate_info.iteration;
        record.parking_seed = candidate_info.parking_seed;
        record.feasible = candidate_summary.all_tasks_succeeded;
        record.makespan = candidate_summary.makespan;
        record.successful_tasks = candidate_summary.successful_tasks;
        record.failed_tasks = candidate_summary.failed_tasks;
        record.order_description =
            preview_task_order(candidate_info.plan, loadedSequence, loadedSequence.size());
        sequence_trial_records.push_back(std::move(record));

        bool learned_from_failure = false;
        if (!outcome.has_partial ||
            is_preferred_search_result(candidate_summary, outcome.best_partial))
        {
          outcome.best_partial = candidate_summary;
          outcome.best_partial.label = label;
          outcome.has_partial = true;
          if (runtime_options.enable_order_constraint_learning &&
              !candidate_summary.all_tasks_succeeded)
          {
            learn_order_constraints_from_failed_summary(
                candidate_summary, method_order_learning_reference_summary.plan,
                loadedSequence, method_constraints);
            learned_from_failure = true;
            std::cout << "[Search][" << progress_tag << " " << candidate_info.iteration
                      << "] new best partial candidate recorded." << std::endl;
          }
        }

        if (candidate_summary.all_tasks_succeeded &&
            (!outcome.has_feasible ||
             is_better_run(candidate_summary, outcome.best_feasible)))
        {
          outcome.best_feasible = candidate_summary;
          outcome.best_feasible.label = label;
          outcome.has_feasible = true;
          maybe_update_method_order_learning_reference(candidate_summary);
          std::cout << "[Search][" << progress_tag << " " << candidate_info.iteration
                    << "] new feasible-best candidate recorded." << std::endl;
        }
        else if (runtime_options.enable_order_constraint_learning &&
                 !candidate_summary.all_tasks_succeeded && !learned_from_failure)
        {
          learn_order_constraints_from_failed_summary(
              candidate_summary, method_order_learning_reference_summary.plan,
              loadedSequence, method_constraints);
        }
      }
    }

    int feasible_count = static_cast<int>(
        std::count_if(sequence_trial_records.begin(), sequence_trial_records.end(),
                      [&](const SearchTrialRecord &record)
                      { return record.search_label == label && record.feasible; }));
    int total_count = static_cast<int>(
        std::count_if(sequence_trial_records.begin(), sequence_trial_records.end(),
                      [&](const SearchTrialRecord &record)
                      { return record.search_label == label; }));

    std::vector<SearchTrialRecord> method_records;
    method_records.reserve(static_cast<std::size_t>(total_count));
    for (const auto &record : sequence_trial_records)
    {
      if (record.search_label == label)
        method_records.push_back(record);
    }

    std::cout << "[Search][" << progress_tag << "] Feasible shuffled plans: "
              << feasible_count << "/" << total_count
              << ", makespans={" << format_feasible_makespan_list(method_records)
              << "}" << std::endl;
    if (out_enforced_constraint_count)
      *out_enforced_constraint_count = method_constraints.enforced_count;

    return outcome;
  };

  const std::string local_sequence_label = "local-edit-task-sequence";
  auto local_sequence_outcome = run_sequence_method(
      local_sequence_label,
      "Local task-sequence search (swap/relocate edits)",
      "SeqLocal",
      runtime_options.local_sequence_search_iterations,
      mutate_sequence_plan,
      &local_sequence_enforced_constraints);

  auto full_shuffle_outcome = run_sequence_method(
      sequence_label,
      "Randomized task-sequence search (full random shuffle)",
      "SeqFull",
      runtime_options.shuffle_sequence_search_iterations,
      mutate_full_shuffle_sequence_plan,
      &full_shuffle_enforced_constraints);

  SequenceSearchOutcome lns_outcome;
  lns_outcome.best_feasible = make_placeholder_summary(lns_label, greedy_plan);
  lns_outcome.best_partial = make_placeholder_summary(lns_label, greedy_plan);

  const AllocationRunSummary *lns_seed_summary = &greedy_summary;
  if (assignment_has_feasible_candidate &&
      is_better_run(assignment_best_feasible, *lns_seed_summary))
  {
    lns_seed_summary = &assignment_best_feasible;
  }
  if (local_sequence_outcome.has_feasible &&
      is_better_run(local_sequence_outcome.best_feasible, *lns_seed_summary))
  {
    lns_seed_summary = &local_sequence_outcome.best_feasible;
  }
  if (full_shuffle_outcome.has_feasible &&
      is_better_run(full_shuffle_outcome.best_feasible, *lns_seed_summary))
  {
    lns_seed_summary = &full_shuffle_outcome.best_feasible;
  }

  if (runtime_options.lns_iterations > 0 && lns_seed_summary->all_tasks_succeeded &&
      loadedSequence.size() > 1 && !robot_names.empty())
  {
    std::cout << "[Search] Adaptive LNS from "
              << lns_seed_summary->label << "..." << std::endl;

    AllocationRunSummary lns_current = *lns_seed_summary;
    lns_current.label = lns_label;
    lns_outcome.best_feasible = lns_current;
    lns_outcome.best_partial = lns_current;
    lns_outcome.has_feasible = true;
    lns_outcome.has_partial = true;
    LearnedOrderConstraints lns_learned_order_constraints =
        make_learned_order_constraints(loadedSequence.size());
    AllocationRunSummary lns_order_learning_reference_summary = lns_current;
    auto maybe_update_lns_order_learning_reference =
        [&](const AllocationRunSummary &summary)
    {
      if (summary.all_tasks_succeeded &&
          is_better_run(summary, lns_order_learning_reference_summary))
      {
        lns_order_learning_reference_summary = summary;
      }
    };

    double destroy_fraction = 0.10;
    int no_improve_iterations = 0;
    std::mt19937 lns_rng(
        mix_seed(runtime_options.base_random_seed, 0x1EA5E123u));

    for (int iter = 0; iter < runtime_options.lns_iterations;)
    {
      AllocationRunSummary batch_base_summary = lns_current;
      AllocationScenarioPlan batch_base_plan = lns_current.plan;
      const LearnedOrderConstraints &batch_constraints =
          runtime_options.enable_order_constraint_learning
              ? lns_learned_order_constraints
              : disabled_order_constraints;
      auto batch_destroy_scores =
          compute_task_destroy_scores(batch_base_summary, loadedSequence);

      std::vector<LnsSearchCandidate> batch_candidates;
      batch_candidates.reserve(static_cast<std::size_t>(lns_batch_size));

      int batch_end = std::min(runtime_options.lns_iterations,
                               iter + lns_batch_size);
      for (; iter < batch_end; ++iter)
      {
        LnsSearchCandidate candidate_info;
        candidate_info.iteration = iter + 1;
        candidate_info.destroy_fraction = destroy_fraction;
        candidate_info.diversification = ((candidate_info.iteration % 50) == 0);

        std::size_t destroy_count = static_cast<std::size_t>(std::round(
            loadedSequence.size() *
            (candidate_info.diversification ? 0.60 : destroy_fraction)));
        destroy_count = std::max<std::size_t>(1, destroy_count);
        destroy_count = std::min<std::size_t>(destroy_count, loadedSequence.size() - 1);

        candidate_info.destroy_scores = batch_destroy_scores;
        if (candidate_info.diversification)
        {
          candidate_info.destroy_operator = "random-large";
          candidate_info.destroyed_tasks = destroy_random_tasks(
              loadedSequence.size(), destroy_count, lns_rng);
        }
        else
        {
          std::uniform_real_distribution<double> op_pick(0.0, 1.0);
          double picked = op_pick(lns_rng);
          if (picked < 0.55)
          {
            candidate_info.destroy_operator = "wait-bottleneck";
            candidate_info.destroyed_tasks = destroy_wait_neighborhood_tasks(
                batch_base_plan, batch_base_summary, loadedSequence,
                destroy_count, lns_rng);
          }
          else if (picked < 0.85)
          {
            candidate_info.destroy_operator = "critical-tail";
            candidate_info.destroyed_tasks = destroy_critical_suffix_tasks(
                batch_base_summary, loadedSequence, destroy_count, lns_rng);
          }
          else
          {
            candidate_info.destroy_operator = "random";
            candidate_info.destroyed_tasks = destroy_random_tasks(
                loadedSequence.size(), destroy_count, lns_rng);
          }
        }

        candidate_info.parking_seed = lns_rng();
        candidate_info.repair_seed = lns_rng();

        std::cout << "[LNS " << candidate_info.iteration << "/"
                  << runtime_options.lns_iterations << "] "
                  << "op=" << candidate_info.destroy_operator
                  << " k=" << candidate_info.destroyed_tasks.size()
                  << " seed=" << candidate_info.parking_seed;
        if (runtime_options.lns_threads > 1)
        {
          std::cout << " queued";
        }
        std::cout << std::endl;

        batch_candidates.push_back(std::move(candidate_info));
      }

      auto batch_results = evaluate_lns_batch(
          loadedSequence, runtime_options, batch_base_plan, batch_base_summary,
          batch_constraints, batch_candidates, robot_names, lns_label);

      for (std::size_t batch_idx = 0; batch_idx < batch_candidates.size(); ++batch_idx)
      {
        const auto &candidate_info = batch_candidates[batch_idx];
        AllocationRunSummary candidate = batch_results[batch_idx];
        candidate.label = lns_label;

        if (runtime_options.enable_order_constraint_learning &&
            !candidate.all_tasks_succeeded)
        {
          learn_order_constraints_from_failed_summary(
              candidate, lns_order_learning_reference_summary.plan,
              loadedSequence, lns_learned_order_constraints);
        }

        if (!lns_outcome.has_partial ||
            is_preferred_search_result(candidate, lns_outcome.best_partial))
        {
          lns_outcome.best_partial = candidate;
          lns_outcome.best_partial.label = lns_label;
          lns_outcome.has_partial = true;
        }

        bool improved_global = false;
        if (candidate.all_tasks_succeeded &&
            (!lns_outcome.has_feasible ||
             is_better_run(candidate, lns_outcome.best_feasible)))
        {
          lns_outcome.best_feasible = candidate;
          lns_outcome.best_feasible.label = lns_label;
          lns_outcome.has_feasible = true;
          improved_global = true;
          maybe_update_lns_order_learning_reference(candidate);
        }

        bool accepted_current = false;
        std::string accept_reason = "reject";
        if (is_preferred_search_result(candidate, lns_current))
        {
          lns_current = candidate;
          lns_current.label = lns_label;
          accepted_current = true;
          accept_reason = improved_global ? "improved" : "accept";
        }
        else if (candidate.all_tasks_succeeded && lns_current.all_tasks_succeeded)
        {
          std::uniform_real_distribution<double> walk_accept_dist(0.0, 1.0);
          if (walk_accept_dist(lns_rng) < 0.10)
          {
            lns_current = candidate;
            lns_current.label = lns_label;
            accepted_current = true;
            accept_reason = "walk";
          }
        }

        if (improved_global)
        {
          destroy_fraction = 0.10;
          no_improve_iterations = 0;
        }
        else
        {
          ++no_improve_iterations;
          if (no_improve_iterations % 30 == 0)
          {
            destroy_fraction = std::min(0.50, destroy_fraction + 0.05);
          }
        }

        std::cout << "[LNS " << candidate_info.iteration << "/"
                  << runtime_options.lns_iterations << "] "
                  << "op=" << candidate_info.destroy_operator
                  << " k=" << candidate_info.destroyed_tasks.size()
                  << " feas=" << (candidate.all_tasks_succeeded ? 1 : 0);
        if (candidate.all_tasks_succeeded)
        {
          std::cout << " mk=" << std::fixed << std::setprecision(2)
                    << candidate.makespan << "s";
        }
        else
        {
          std::cout << " suc=" << candidate.successful_tasks << "/"
                    << (candidate.successful_tasks + candidate.failed_tasks);
        }

        if (lns_outcome.has_feasible)
        {
          std::cout << " best=" << std::fixed << std::setprecision(2)
                    << lns_outcome.best_feasible.makespan << "s";
        }
        else
        {
          std::cout << " best=none";
        }

        std::cout << " acc=" << accept_reason;
        if (accepted_current && accept_reason == "walk")
        {
          std::cout << " cur=" << std::fixed << std::setprecision(2)
                    << lns_current.makespan << "s";
        }
        if (runtime_options.lns_threads > 1)
        {
          std::cout << " batch=" << batch_candidates.size();
        }
        std::cout << std::endl;
      }
    }
    lns_enforced_constraints = lns_learned_order_constraints.enforced_count;
  }
  else
  {
    lns_enforced_constraints = 0;
    std::cout << "[Search] Skipping adaptive LNS." << std::endl;
  }

  AllocationRunSummary local_sequence_report =
      local_sequence_outcome.has_feasible
          ? local_sequence_outcome.best_feasible
          : local_sequence_outcome.best_partial;

  AllocationRunSummary sequence_report =
      full_shuffle_outcome.has_feasible
          ? full_shuffle_outcome.best_feasible
          : full_shuffle_outcome.best_partial;

  AllocationRunSummary lns_report =
      lns_outcome.has_feasible
          ? lns_outcome.best_feasible
          : lns_outcome.best_partial;

  if (runtime_options.enable_order_constraint_learning)
  {
    std::cout << "[Learn][Order] " << local_sequence_label
              << " enforced precedence constraints: "
              << local_sequence_enforced_constraints << std::endl;
    std::cout << "[Learn][Order] " << sequence_label
              << " enforced precedence constraints: "
              << full_shuffle_enforced_constraints << std::endl;
    std::cout << "[Learn][Order] " << lns_label
              << " enforced precedence constraints: "
              << lns_enforced_constraints << std::endl;
  }
  else
  {
    std::cout << "[Learn][Order] disabled" << std::endl;
  }
  std::cout << "[Compare] Makespan summary" << std::endl;
  print_comparison_line(greedy_summary, greedy_summary.makespan);
  print_search_method_summary(assignment_label,
                              assignment_has_feasible_candidate,
                              assignment_best_feasible,
                              assignment_has_partial_candidate,
                              assignment_best_partial,
                              greedy_summary.makespan);
  print_search_method_summary(local_sequence_label,
                              local_sequence_outcome.has_feasible,
                              local_sequence_outcome.best_feasible,
                              local_sequence_outcome.has_partial,
                              local_sequence_outcome.best_partial,
                              greedy_summary.makespan);
  print_search_method_summary(sequence_label,
                              full_shuffle_outcome.has_feasible,
                              full_shuffle_outcome.best_feasible,
                              full_shuffle_outcome.has_partial,
                              full_shuffle_outcome.best_partial,
                              greedy_summary.makespan);
  print_search_method_summary(lns_label,
                              lns_outcome.has_feasible,
                              lns_outcome.best_feasible,
                              lns_outcome.has_partial,
                              lns_outcome.best_partial,
                              greedy_summary.makespan);

  std::vector<AllocationRunSummary> summaries = {
      greedy_summary,
      assignment_report,
      local_sequence_report,
      sequence_report,
      lns_report};

  for (const auto &summary : summaries)
  {
    std::string scenario_csv = std::string(CMAKE_SOURCE_DIR) + "/task_execution_log_" +
                               sanitize_filename_component(summary.label) + ".csv";
    write_task_csv_log(scenario_csv, summary.task_rows);
  }

  std::string comparison_csv =
      std::string(CMAKE_SOURCE_DIR) + "/allocation_search_summary.csv";
  write_allocation_search_summary_csv(comparison_csv, summaries,
                                      greedy_summary.makespan);

  if (!sequence_trial_records.empty())
  {
    std::string trial_csv =
        std::string(CMAKE_SOURCE_DIR) + "/task_shuffle_trial_log.csv";
    write_search_trial_records_csv(trial_csv, sequence_trial_records);
  }

  const AllocationRunSummary *best_summary = &greedy_summary;
  if (assignment_has_feasible_candidate ||
      local_sequence_outcome.has_feasible ||
      full_shuffle_outcome.has_feasible ||
      lns_outcome.has_feasible)
  {
    if (assignment_has_feasible_candidate &&
        is_better_run(assignment_best_feasible, *best_summary))
    {
      best_summary = &assignment_best_feasible;
    }
    if (local_sequence_outcome.has_feasible &&
        is_better_run(local_sequence_outcome.best_feasible, *best_summary))
    {
      best_summary = &local_sequence_outcome.best_feasible;
    }
    if (full_shuffle_outcome.has_feasible &&
        is_better_run(full_shuffle_outcome.best_feasible, *best_summary))
    {
      best_summary = &full_shuffle_outcome.best_feasible;
    }
    if (lns_outcome.has_feasible &&
        is_better_run(lns_outcome.best_feasible, *best_summary))
    {
      best_summary = &lns_outcome.best_feasible;
    }
  }

  const double lns_best_makespan =
      lns_outcome.has_feasible
          ? lns_outcome.best_feasible.makespan
          : std::numeric_limits<double>::infinity();

  std::string instance_record_csv =
      std::string(CMAKE_SOURCE_DIR) + "/results/mars_instance_record_" +
      sanitize_filename_component(instance_info.file_name) + ".csv";
  write_instance_run_record_csv(
      instance_record_csv,
      instance_info,
      relopush_single_robot_makespan,
      greedy_summary.makespan,
      lns_best_makespan,
      runtime_options.lns_iterations,
      best_summary->label,
      best_summary->makespan);

  std::cout << "[Final] Replaying best scenario: "
            << best_summary->label;
  if (runtime_options.enable_visualization)
    std::cout << " (with visualization)";
  else
    std::cout << " (headless)";
  std::cout << std::endl;
  auto best_executed = execute_allocation_scenario(
      loadedSequence, runtime_options, best_summary->plan,
      best_summary->label, best_summary->parking_seed, true);

  std::string csv_path = std::string(CMAKE_SOURCE_DIR) + "/task_execution_log.csv";
  write_task_csv_log(csv_path, best_executed.summary.task_rows);
  log_timetable_orientation_diagnostics(best_executed.summary.label,
                                        best_executed.timetable,
                                        best_executed.entities,
                                        best_executed.params);

  export_result_summary_figure(runtime_options, instance_info,
                               best_executed.summary.label,
                               best_executed.timetable,
                               best_executed.entities,
                               best_executed.params);

  notify_relopush_if_needed(
      best_executed.summary.all_tasks_succeeded,
      best_executed.summary.all_tasks_succeeded ? "best-scenario-complete"
                                                : "best-scenario-failed");

  if (runtime_options.enable_visualization)
  {
    show_results(argc, argv, best_executed.timetable, best_executed.entities,
                 best_executed.params);
  }

  return best_executed.summary.all_tasks_succeeded ? 0 : 1;
}

#ifndef PHASTAR_PUSH_NO_MAIN
int main(int argc, char **argv)
{
  std::cout << "== Pushing ==" << std::endl;
  return phastar_push_demo_main(argc, argv);
}
#endif
