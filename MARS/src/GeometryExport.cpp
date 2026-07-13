#include <GeometryExport.h>
#include <DqnFeaturesV2.h>
#include <ReloPush/TaskAllocation.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{

void append_state_path(const ReloPush::StatePath &path, bool is_pushing,
                       std::vector<PathPoint> &out)
{
  for (const auto &s : path)
  {
    PathPoint p;
    p.x = s.x;
    p.y = s.y;
    p.yaw = s.yaw;
    p.is_pushing = is_pushing;
    out.push_back(p);
  }
}

} // namespace

std::vector<PathPoint> extract_ordered_task_path(const FinalAllocation &fa)
{
  std::vector<PathPoint> out;

  // Note: EdgePath::getFirstWaypoint()/getLastWaypoint()/getLength() are not
  // const, but shared_ptr indirection (obsReloPaths, EdgeData::paths) does
  // not propagate const from the enclosing `const FinalAllocation&` -- same
  // reasoning as DqnFeaturesV2.cpp's extract_task_geoms().
  auto consume_subpath = [&](EdgePath &ep)
  {
    auto state_path = ep.toStatePath(DqnV2::kCorridorSampleSpacing);
    if (!state_path || state_path->empty())
      return;
    append_state_path(*state_path, ep.is_pushing, out);
  };

  if (fa.obsReloPaths)
  {
    for (auto &ep : *fa.obsReloPaths)
      consume_subpath(ep);
  }
  for (const auto &edge : fa.paths)
  {
    for (const auto &ep_ptr : edge.paths)
    {
      if (ep_ptr)
        consume_subpath(*ep_ptr);
    }
  }

  return out;
}

std::vector<PathPoint> resample_to_k_points(
    const std::vector<PathPoint> &ordered_path, int k)
{
  std::vector<PathPoint> out;
  if (ordered_path.empty())
    return out;
  if (k <= 1)
  {
    out.push_back(ordered_path.front());
    return out;
  }

  std::vector<double> cum(ordered_path.size(), 0.0);
  for (std::size_t i = 1; i < ordered_path.size(); ++i)
  {
    cum[i] = cum[i - 1] + std::hypot(ordered_path[i].x - ordered_path[i - 1].x,
                                     ordered_path[i].y - ordered_path[i - 1].y);
  }
  const double total_length = cum.back();

  if (total_length < 1e-9)
  {
    out.assign(static_cast<std::size_t>(k), ordered_path.front());
    return out;
  }

  out.reserve(static_cast<std::size_t>(k));
  std::size_t seg = 0;
  const std::size_t max_seg = ordered_path.size() - 2;
  for (int i = 0; i < k; ++i)
  {
    const double target =
        total_length * static_cast<double>(i) / static_cast<double>(k - 1);
    while (seg < max_seg && cum[seg + 1] < target)
      ++seg;

    const double seg_start = cum[seg];
    const double seg_end = cum[seg + 1];
    const double seg_len = seg_end - seg_start;
    const double frac = seg_len > 1e-9 ? (target - seg_start) / seg_len : 0.0;

    PathPoint p;
    p.x = ordered_path[seg].x + frac * (ordered_path[seg + 1].x - ordered_path[seg].x);
    p.y = ordered_path[seg].y + frac * (ordered_path[seg + 1].y - ordered_path[seg].y);
    // yaw/is_pushing come from the arclength-nearer endpoint (tie -> earlier).
    const bool nearer_is_end = frac > 0.5;
    p.yaw = nearer_is_end ? ordered_path[seg + 1].yaw : ordered_path[seg].yaw;
    p.is_pushing = nearer_is_end ? ordered_path[seg + 1].is_pushing : ordered_path[seg].is_pushing;

    out.push_back(p);
  }
  return out;
}

std::string json_escape_string(const std::string &s)
{
  std::string out;
  out.reserve(s.size());
  for (unsigned char c : s)
  {
    switch (c)
    {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (c < 0x20)
      {
        std::ostringstream oss;
        oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
            << static_cast<int>(c);
        out += oss.str();
      }
      else
      {
        out += static_cast<char>(c);
      }
    }
  }
  return out;
}

bool export_instance_geometry(
    const std::string &out_path,
    const std::string &family, int index,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<RobotMeta> &robot_metas,
    int k)
{
  if (loaded_sequence.empty() || robot_metas.empty())
  {
    std::cerr << "[GeometryExport] Empty loaded_sequence/robot_metas; nothing to export."
              << std::endl;
    return false;
  }

  const std::filesystem::path fs_path(out_path);
  const std::filesystem::path parent = fs_path.parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent);

  std::ofstream out(out_path);
  if (!out.is_open())
  {
    std::cerr << "[GeometryExport] Failed to open output file: " << out_path
              << std::endl;
    return false;
  }

  out << std::setprecision(6);

  const auto &boundary = loaded_sequence[0].snapshot.parameters.boundary;
  const auto task_geoms = DqnV2::extract_task_geoms(loaded_sequence, robot_metas[0]);
  const std::size_t n = loaded_sequence.size();

  out << "{\n";
  out << "  \"family\": \"" << json_escape_string(family) << "\",\n";
  out << "  \"index\": " << index << ",\n";
  out << "  \"workspace\": {\"xMin\": " << boundary.xMin
      << ", \"xMax\": " << boundary.xMax << ", \"yMin\": " << boundary.yMin
      << ", \"yMax\": " << boundary.yMax << "},\n";

  out << "  \"robots\": [";
  for (std::size_t i = 0; i < robot_metas.size(); ++i)
  {
    if (i > 0)
      out << ", ";
    const auto &meta = robot_metas[i];
    out << "{\"name\": \"" << json_escape_string(meta.name) << "\", \"x\": "
        << meta.initial_pose.x << ", \"y\": " << meta.initial_pose.y
        << ", \"yaw\": " << meta.initial_pose.yaw << "}";
  }
  out << "],\n";

  out << "  \"reference_order\": [";
  for (std::size_t i = 0; i < n; ++i)
  {
    if (i > 0)
      out << ", ";
    out << i;
  }
  out << "],\n";

  out << "  \"tasks\": [\n";
  for (std::size_t i = 0; i < n; ++i)
  {
    const FinalAllocation &fa = loaded_sequence[i];
    const auto path = resample_to_k_points(extract_ordered_task_path(fa), k);

    out << "    {\n";
    out << "      \"task_id\": " << i << ",\n";
    out << "      \"start\": {\"x\": " << fa.startPose.x << ", \"y\": "
        << fa.startPose.y << ", \"yaw\": " << fa.startPose.yaw << "},\n";
    out << "      \"goal\": {\"x\": " << fa.goalPose.x << ", \"y\": "
        << fa.goalPose.y << ", \"yaw\": " << fa.goalPose.yaw << "},\n";
    out << "      \"tau_fixed\": " << task_geoms[i].tau_fixed << ",\n";
    out << "      \"path\": [";
    for (std::size_t j = 0; j < path.size(); ++j)
    {
      if (j > 0)
        out << ", ";
      out << "{\"x\": " << path[j].x << ", \"y\": " << path[j].y
          << ", \"yaw\": " << path[j].yaw << ", \"is_pushing\": "
          << (path[j].is_pushing ? "true" : "false") << "}";
    }
    out << "]\n";
    out << "    }" << (i + 1 < n ? "," : "") << "\n";
  }
  out << "  ]\n";
  out << "}\n";

  return true;
}
