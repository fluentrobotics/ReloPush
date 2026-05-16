/*****************************************************************
 * Result Summary Figure (QPainter)
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <CsvLogging.h>  // for default_result_summary_path, sanitize_filename_component
#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <PHAstar/Entities.h>
#include <PHAstar/Utils.h>
#include <ReloPush/FinalSequenceHandoff.h>

#include <QApplication>
#include <QFont>
#include <QImage>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

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
