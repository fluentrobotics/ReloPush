#ifndef RELOPUSH_BOSS_DIAGNOSTICS_HPP
#define RELOPUSH_BOSS_DIAGNOSTICS_HPP

#include <ReloPush/TaskAllocation.hpp>
#include <ReloPush/config.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ReloPushBossDiagnostics
{
struct Session
{
    bool initialized = false;
    int current_depth = 0;
    std::string stem;
    std::string summary_path;
    std::string trace_path;
    std::string final_csv_path;
    std::ofstream summary;
    std::ofstream trace;
};

inline Session &session()
{
    static Session s;
    return s;
}

inline std::string sanitize_filename_component(const std::string &label)
{
    std::string out;
    out.reserve(label.size());
    for (char c : label)
    {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (std::isalnum(uc))
            out.push_back(static_cast<char>(std::tolower(uc)));
        else
            out.push_back('_');
    }
    while (!out.empty() && out.front() == '_')
        out.erase(out.begin());
    while (!out.empty() && out.back() == '_')
        out.pop_back();
    return out.empty() ? "instance" : out;
}

inline std::string csv_escape(const std::string &field)
{
    bool quote = false;
    for (char c : field)
    {
        if (c == '"' || c == ',' || c == '\n' || c == '\r')
        {
            quote = true;
            break;
        }
    }
    if (!quote)
        return field;

    std::string out = "\"";
    for (char c : field)
    {
        if (c == '"')
            out += "\"\"";
        else
            out.push_back(c);
    }
    out.push_back('"');
    return out;
}

inline std::uint64_t fnv1a_append(std::uint64_t hash,
                                  const void *data,
                                  std::size_t size)
{
    constexpr std::uint64_t kPrime = 1099511628211ull;
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (std::size_t i = 0; i < size; ++i)
    {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= kPrime;
    }
    return hash;
}

inline std::uint64_t fnv1a_string(const std::string &value)
{
    return fnv1a_append(1469598103934665603ull,
                        value.data(),
                        value.size());
}

inline std::uint64_t fnv1a_file(const std::string &path)
{
    std::ifstream ifs(path, std::ios::binary);
    std::uint64_t hash = 1469598103934665603ull;
    if (!ifs)
        return hash;

    char buffer[4096];
    while (ifs)
    {
        ifs.read(buffer, sizeof(buffer));
        const std::streamsize count = ifs.gcount();
        if (count > 0)
            hash = fnv1a_append(hash, buffer, static_cast<std::size_t>(count));
    }
    return hash;
}

inline std::string hex_u64(std::uint64_t value)
{
    std::ostringstream oss;
    oss << std::hex << std::setw(16) << std::setfill('0') << value;
    return oss.str();
}

inline void canonical_append(std::ostringstream &oss, double value)
{
    oss << std::setprecision(17) << value;
}

inline void canonical_append_state(std::ostringstream &oss,
                                   const ReloPush::State &state)
{
    canonical_append(oss, state.x);
    oss << ",";
    canonical_append(oss, state.y);
    oss << ",";
    canonical_append(oss, state.yaw);
    oss << ",";
    canonical_append(oss, state.time);
    oss << ",";
    canonical_append(oss, state.vel);
    oss << "," << (state.is_pushing ? 1 : 0);
}

inline void canonical_append_object(std::ostringstream &oss,
                                    const ObjectInfo &object)
{
    oss << object.name << ",";
    canonical_append(oss, object.x);
    oss << ",";
    canonical_append(oss, object.y);
    oss << ",";
    canonical_append(oss, object.nominalOrientation);
    oss << "," << object.numberOfSides << ",";
    canonical_append(oss, object.enclosingRadius);
}

inline std::string state_to_string(const ReloPush::State &state)
{
    std::ostringstream oss;
    canonical_append_state(oss, state);
    return oss.str();
}

inline std::string object_to_string(const ObjectInfo &object)
{
    std::ostringstream oss;
    canonical_append_object(oss, object);
    return oss.str();
}

inline std::string map_raw_order(const ObjectMap &map)
{
    std::ostringstream oss;
    bool first = true;
    for (const auto &kv : map)
    {
        if (!first)
            oss << ";";
        first = false;
        oss << kv.first;
    }
    return oss.str();
}

inline std::string pair_map_raw_order(
    const std::unordered_map<std::string, ObjectGoalPair> &pairs)
{
    std::ostringstream oss;
    bool first = true;
    for (const auto &kv : pairs)
    {
        if (!first)
            oss << ";";
        first = false;
        oss << kv.first << "->" << kv.second.goalName;
    }
    return oss.str();
}

inline std::string state_map_raw_order(
    const std::unordered_map<std::string, ReloPush::State> &map)
{
    std::ostringstream oss;
    bool first = true;
    for (const auto &kv : map)
    {
        if (!first)
            oss << ";";
        first = false;
        oss << kv.first;
    }
    return oss.str();
}

inline std::string sorted_object_map_string(const ObjectMap &map)
{
    std::vector<std::pair<std::string, ObjectInfo>> entries(map.begin(), map.end());
    std::sort(entries.begin(), entries.end(),
              [](const auto &a, const auto &b)
              { return a.first < b.first; });
    std::ostringstream oss;
    bool first = true;
    for (const auto &kv : entries)
    {
        if (!first)
            oss << ";";
        first = false;
        oss << kv.first << "=" << object_to_string(kv.second);
    }
    return oss.str();
}

inline std::string sorted_pair_map_string(
    const std::unordered_map<std::string, ObjectGoalPair> &pairs)
{
    std::vector<std::pair<std::string, ObjectGoalPair>> entries(pairs.begin(), pairs.end());
    std::sort(entries.begin(), entries.end(),
              [](const auto &a, const auto &b)
              { return a.first < b.first; });
    std::ostringstream oss;
    bool first = true;
    for (const auto &kv : entries)
    {
        if (!first)
            oss << ";";
        first = false;
        oss << kv.first << "->" << kv.second.goalName;
    }
    return oss.str();
}

inline std::string sorted_state_map_string(
    const std::unordered_map<std::string, ReloPush::State> &map)
{
    std::vector<std::pair<std::string, ReloPush::State>> entries(map.begin(), map.end());
    std::sort(entries.begin(), entries.end(),
              [](const auto &a, const auto &b)
              { return a.first < b.first; });
    std::ostringstream oss;
    bool first = true;
    for (const auto &kv : entries)
    {
        if (!first)
            oss << ";";
        first = false;
        oss << kv.first << "=" << state_to_string(kv.second);
    }
    return oss.str();
}

inline std::string path_hash(const ReloPush::StatePathPtr &path)
{
    std::ostringstream oss;
    if (!path)
    {
        oss << "null";
        return hex_u64(fnv1a_string(oss.str()));
    }

    oss << "size=" << path->size();
    for (const auto &state : *path)
    {
        oss << "|";
        canonical_append_state(oss, state);
    }
    return hex_u64(fnv1a_string(oss.str()));
}

inline std::size_t path_size(const ReloPush::StatePathPtr &path)
{
    return path ? path->size() : 0;
}

inline std::string edge_path_hash(const EdgePathPtr &edge_path)
{
    if (!edge_path)
        return hex_u64(fnv1a_string("null_edge_path"));
    return path_hash(edge_path->toStatePath());
}

inline void canonical_append_vertex(std::ostringstream &oss,
                                    const VertexData &vertex)
{
    oss << static_cast<int>(vertex.type) << ","
        << vertex.name << ","
        << vertex.orientationIndex << ",";
    canonical_append(oss, vertex.nominalOrientation);
    oss << ",";
    canonical_append(oss, vertex.x);
    oss << ",";
    canonical_append(oss, vertex.y);
    oss << "," << vertex.numberOfSides << ",";
    canonical_append(oss, vertex.radius);
}

inline void canonical_append_edge(std::ostringstream &oss,
                                  const EdgeData &edge)
{
    canonical_append(oss, edge.weight);
    oss << "|src:";
    canonical_append_vertex(oss, edge.srcVertexData);
    oss << "|sink:";
    canonical_append_vertex(oss, edge.sinkVertexData);
    oss << "|mode:" << static_cast<int>(edge.mode)
        << "|pre:" << (edge.preRelo.used ? 1 : 0) << ",";
    canonical_append(oss, edge.preRelo.xRelocated_robot);
    oss << ",";
    canonical_append(oss, edge.preRelo.yRelocated_robot);
    oss << ",";
    canonical_append(oss, edge.preRelo.yawReloacted_robot);
    oss << ",";
    canonical_append(oss, edge.preRelo.xRelocated_object);
    oss << ",";
    canonical_append(oss, edge.preRelo.yRelocated_object);
    oss << ",";
    canonical_append(oss, edge.preRelo.yawRelocated_object);
    oss << ",";
    canonical_append(oss, edge.preRelo.extraCost);
    oss << "," << edge.preRelo.relocatingIndex
        << "," << static_cast<int>(edge.preRelo.reason);
    for (const auto &path : edge.paths)
        oss << "|path:" << edge_path_hash(path);
}

inline std::string edge_data_hash(const EdgeData &edge)
{
    std::ostringstream oss;
    canonical_append_edge(oss, edge);
    return hex_u64(fnv1a_string(oss.str()));
}

inline std::string graph_hash(const Graph &graph)
{
    std::ostringstream oss;
    oss << "V=" << boost::num_vertices(graph) << "|E=" << boost::num_edges(graph);
    auto vertices = boost::vertices(graph);
    for (auto it = vertices.first; it != vertices.second; ++it)
    {
        oss << "|v" << *it << ":";
        canonical_append_vertex(oss, graph[*it]);
    }
    auto edges = boost::edges(graph);
    for (auto it = edges.first; it != edges.second; ++it)
    {
        oss << "|e" << boost::source(*it, graph) << "->"
            << boost::target(*it, graph) << ":";
        canonical_append_edge(oss, graph[*it]);
    }
    return hex_u64(fnv1a_string(oss.str()));
}

inline std::string candidates_preview(const std::vector<LowestCostInfo> &candidates,
                                      std::size_t limit = 30)
{
    std::ostringstream oss;
    oss << std::setprecision(17);
    const std::size_t n = std::min(limit, candidates.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        if (i > 0)
            oss << ";";
        const auto &c = candidates[i];
        oss << i << ":" << c.objectName << "->" << c.goalName
            << "[" << c.row << "," << c.col << "]=" << c.cost;
    }
    if (candidates.size() > n)
        oss << ";...+" << (candidates.size() - n);
    return oss.str();
}

inline std::string pair_result_preview(const PairCostResult &result,
                                       std::size_t limit = 10)
{
    std::ostringstream oss;
    oss << std::setprecision(17);
    if (!result.matrixResult)
        return "null_matrix";
    const auto &entries = result.matrixResult->sortedEntries;
    const std::size_t n = std::min(limit, entries.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        if (i > 0)
            oss << ";";
        const auto &entry = entries[i];
        oss << i << "[" << entry.row << "," << entry.col << "]=" << entry.cost;
    }
    if (entries.size() > n)
        oss << ";...+" << (entries.size() - n);
    return oss.str();
}

inline std::string trace_value(const std::string &value)
{
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (char c : value)
    {
        if (c == '"' || c == '\\')
            out.push_back('\\');
        if (c == '\n' || c == '\r')
            out.push_back(' ');
        else
            out.push_back(c);
    }
    out.push_back('"');
    return out;
}

inline void trace_event(
    const std::string &event,
    const std::vector<std::pair<std::string, std::string>> &fields)
{
    auto &s = session();
    if (!s.initialized || !s.trace.is_open())
        return;

    s.trace << "event=" << event;
    for (const auto &field : fields)
        s.trace << " " << field.first << "=" << trace_value(field.second);
    s.trace << "\n";
}

inline void initialize(const std::string &filename,
                       int instance_index,
                       bool use_opt,
                       bool no_init_guess,
                       bool use_dfs,
                       const std::string &input_path,
                       const std::string &selected_line)
{
    auto &s = session();
    if (s.summary.is_open())
        s.summary.close();
    if (s.trace.is_open())
        s.trace.close();

    const std::string diagnostics_dir =
        std::string(CMAKE_SOURCE_DIR) + "/diagnostics";
    std::filesystem::create_directories(diagnostics_dir);
    s.stem = "relopush_boss_repro_" + sanitize_filename_component(filename) +
             "_ind" + std::to_string(instance_index);
    s.summary_path = diagnostics_dir + "/" + s.stem + ".txt";
    s.trace_path = diagnostics_dir + "/" + s.stem + "_trace.txt";
    s.final_csv_path = diagnostics_dir + "/" + s.stem + "_final_sequence.csv";
    s.summary.open(s.summary_path);
    s.trace.open(s.trace_path);
    s.initialized = s.summary.is_open() && s.trace.is_open();
    s.current_depth = 0;

    if (!s.initialized)
    {
        std::cerr << "[ReloPushBossDiag] Failed to open diagnostics under "
                  << diagnostics_dir << std::endl;
        return;
    }

    s.summary << std::fixed << std::setprecision(17);
    s.trace << std::fixed << std::setprecision(17);
    s.summary << "[ReloPushBossReproDiagnostics]\n";
    s.summary << "trace_file=" << s.trace_path << "\n";
    s.summary << "final_sequence_csv=" << s.final_csv_path << "\n";
    s.summary << "input_filename=" << filename << "\n";
    s.summary << "instance_index=" << instance_index << "\n";
    s.summary << "input_path=" << input_path << "\n";
    s.summary << "input_file_fnv1a64=" << hex_u64(fnv1a_file(input_path)) << "\n";
    s.summary << "selected_line_fnv1a64="
              << hex_u64(fnv1a_string(selected_line)) << "\n";
    s.summary << "selected_line_size=" << selected_line.size() << "\n";
    s.summary << "use_opt=" << (use_opt ? 1 : 0) << "\n";
    s.summary << "no_init_guess=" << (no_init_guess ? 1 : 0) << "\n";
    s.summary << "use_dfs=" << (use_dfs ? 1 : 0) << "\n";
    s.summary << "Constants::r_push=" << Constants::r_push << "\n";
    s.summary << "Constants::r_nonpush=" << Constants::r_nonpush << "\n";
    s.summary << "Constants::mapResolution=" << Constants::mapResolution << "\n";
    s.summary << "Constants::prepush_th=" << Constants::prepush_th << "\n";
    s.summary << "Constants::additional_push_dist="
              << Constants::additional_push_dist << "\n";
    s.summary << "Constants::obs_relo_offset=" << Constants::obs_relo_offset << "\n";
    s.summary << "sizeof_float=" << sizeof(float) << "\n";
    s.summary << "sizeof_double=" << sizeof(double) << "\n";
    s.summary << "sizeof_long_double=" << sizeof(long double) << "\n";
    s.summary << "FLT_EVAL_METHOD=" << FLT_EVAL_METHOD << "\n";
#if defined(__clang__)
    s.summary << "compiler=clang " << __clang_version__ << "\n";
#elif defined(__GNUC__)
    s.summary << "compiler=gcc " << __VERSION__ << "\n";
#else
    s.summary << "compiler=unknown\n";
#endif
#if defined(__APPLE__)
    s.summary << "platform_macro=__APPLE__\n";
#elif defined(__linux__)
    s.summary << "platform_macro=__linux__\n";
#else
    s.summary << "platform_macro=unknown\n";
#endif

    trace_event("run_start",
                {{"filename", filename},
                 {"instance_index", std::to_string(instance_index)},
                 {"input_hash", hex_u64(fnv1a_file(input_path))},
                 {"selected_line_hash", hex_u64(fnv1a_string(selected_line))}});
}

inline void set_current_depth(int depth)
{
    session().current_depth = depth;
}

inline void log_parsed_input(
    const ObjectMap &objects,
    const GoalMap &goals,
    const std::vector<ReloPush::State> &robots,
    const std::unordered_map<std::string, ObjectGoalPair> &obj_goal_pairs)
{
    auto &s = session();
    if (!s.initialized)
        return;

    std::ostringstream robot_oss;
    for (std::size_t i = 0; i < robots.size(); ++i)
    {
        if (i > 0)
            robot_oss << ";";
        robot_oss << i << "=" << state_to_string(robots[i]);
    }

    const std::string objects_sorted = sorted_object_map_string(objects);
    const std::string goals_sorted = sorted_object_map_string(goals);
    const std::string pairs_sorted = sorted_pair_map_string(obj_goal_pairs);

    s.summary << "parsed_object_count=" << objects.size() << "\n";
    s.summary << "parsed_goal_count=" << goals.size() << "\n";
    s.summary << "parsed_robot_count=" << robots.size() << "\n";
    s.summary << "parsed_pair_count=" << obj_goal_pairs.size() << "\n";
    s.summary << "objects_raw_order=" << map_raw_order(objects) << "\n";
    s.summary << "goals_raw_order=" << map_raw_order(goals) << "\n";
    s.summary << "pairs_raw_order=" << pair_map_raw_order(obj_goal_pairs) << "\n";
    s.summary << "objects_sorted_fnv1a64=" << hex_u64(fnv1a_string(objects_sorted)) << "\n";
    s.summary << "goals_sorted_fnv1a64=" << hex_u64(fnv1a_string(goals_sorted)) << "\n";
    s.summary << "pairs_sorted_fnv1a64=" << hex_u64(fnv1a_string(pairs_sorted)) << "\n";

    trace_event("parsed_input",
                {{"objects_raw", map_raw_order(objects)},
                 {"goals_raw", map_raw_order(goals)},
                 {"pairs_raw", pair_map_raw_order(obj_goal_pairs)},
                 {"objects_sorted_hash", hex_u64(fnv1a_string(objects_sorted))},
                 {"goals_sorted_hash", hex_u64(fnv1a_string(goals_sorted))},
                 {"pairs_sorted_hash", hex_u64(fnv1a_string(pairs_sorted))},
                 {"robots", robot_oss.str()}});
}

inline void log_search_state(int depth,
                             const ObjectMap &objects,
                             const GoalMap &goals,
                             const std::unordered_map<std::string, ObjectGoalPair> &obj_goal_pairs,
                             const GoalMap &delivered,
                             const ReloPush::State &robot,
                             const Graph &graph)
{
    set_current_depth(depth);
    trace_event("search_state",
                {{"depth", std::to_string(depth)},
                 {"remaining_pairs", std::to_string(obj_goal_pairs.size())},
                 {"objects_raw", map_raw_order(objects)},
                 {"goals_raw", map_raw_order(goals)},
                 {"pairs_raw", pair_map_raw_order(obj_goal_pairs)},
                 {"delivered_raw", map_raw_order(delivered)},
                 {"objects_sorted_hash", hex_u64(fnv1a_string(sorted_object_map_string(objects)))},
                 {"goals_sorted_hash", hex_u64(fnv1a_string(sorted_object_map_string(goals)))},
                 {"pairs_sorted_hash", hex_u64(fnv1a_string(sorted_pair_map_string(obj_goal_pairs)))},
                 {"delivered_sorted_hash", hex_u64(fnv1a_string(sorted_object_map_string(delivered)))},
                 {"robot", state_to_string(robot)},
                 {"graph_vertices", std::to_string(boost::num_vertices(graph))},
                 {"graph_edges", std::to_string(boost::num_edges(graph))},
                 {"graph_hash", graph_hash(graph)}});
}

inline void log_pair_results(int depth, const PairResultsMap &pair_results)
{
    for (const auto &kv : pair_results)
    {
        const auto &result = kv.second;
        const auto entry_count = result.matrixResult
                                     ? result.matrixResult->sortedEntries.size()
                                     : 0;
        trace_event("pair_result",
                    {{"depth", std::to_string(depth)},
                     {"object", result.objectName},
                     {"goal", result.goalName},
                     {"entry_count", std::to_string(entry_count)},
                     {"top_entries", pair_result_preview(result)}});
    }
}

inline void log_candidate_list(int depth,
                               const std::vector<LowestCostInfo> &candidates)
{
    trace_event("candidate_list",
                {{"depth", std::to_string(depth)},
                 {"count", std::to_string(candidates.size())},
                 {"top_candidates", candidates_preview(candidates)}});
}

inline void log_try_candidate(int depth, const LowestCostInfo &candidate)
{
    std::ostringstream cost;
    cost << std::setprecision(17) << candidate.cost;
    trace_event("try_candidate",
                {{"depth", std::to_string(depth)},
                 {"object", candidate.objectName},
                 {"goal", candidate.goalName},
                 {"row", std::to_string(candidate.row)},
                 {"col", std::to_string(candidate.col)},
                 {"cost", cost.str()}});
}

inline void log_try_stage(const std::string &stage,
                          const LowestCostInfo &candidate,
                          const std::string &detail = "")
{
    trace_event("try_stage",
                {{"depth", std::to_string(session().current_depth)},
                 {"stage", stage},
                 {"object", candidate.objectName},
                 {"goal", candidate.goalName},
                 {"row", std::to_string(candidate.row)},
                 {"col", std::to_string(candidate.col)},
                 {"detail", detail}});
}

inline std::string allocation_hash(const FinalAllocation &allocation)
{
    std::ostringstream oss;
    canonical_append_object(oss, allocation.object);
    oss << "|goal:";
    canonical_append_object(oss, allocation.goal);
    oss << "|cost:";
    canonical_append(oss, allocation.cost);
    oss << "|rowcol:" << allocation.row << "," << allocation.col;
    oss << "|vertices:";
    for (const auto &vertex : allocation.vertexChain)
    {
        oss << "[";
        canonical_append_vertex(oss, vertex);
        oss << "]";
    }
    oss << "|first:" << path_hash(allocation.firstApproachPath);
    oss << "|obs:";
    if (allocation.obsReloPaths)
    {
        for (const auto &edge_path : *allocation.obsReloPaths)
            oss << (edge_path.is_pushing ? "P" : "T") << ":"
                << path_hash(edge_path.toStatePath()) << ";";
    }
    oss << "|edges:";
    for (const auto &edge : allocation.paths)
        oss << edge_data_hash(edge) << ";";
    oss << "|edge_transit:";
    for (const auto &path : allocation.edgeTransitPaths)
        oss << path_hash(path) << ";";
    oss << "|obs_update:" << sorted_state_map_string(allocation.obsReloUpdate);
    oss << "|snapshot_mo:" << sorted_object_map_string(allocation.snapshot.mo_list);
    oss << "|snapshot_delivered:" << sorted_object_map_string(allocation.snapshot.delivered_list);
    oss << "|sampled:";
    for (const auto &state : allocation.snapshot.sampledPositions)
    {
        canonical_append_state(oss, state);
        oss << ";";
    }
    return hex_u64(fnv1a_string(oss.str()));
}

inline void log_try_result(int depth,
                           const LowestCostInfo &candidate,
                           bool ok,
                           const FinalAllocation *allocation = nullptr)
{
    std::vector<std::pair<std::string, std::string>> fields = {
        {"depth", std::to_string(depth)},
        {"object", candidate.objectName},
        {"goal", candidate.goalName},
        {"row", std::to_string(candidate.row)},
        {"col", std::to_string(candidate.col)},
        {"ok", ok ? "1" : "0"}};
    if (allocation)
    {
        fields.push_back({"allocation_hash", allocation_hash(*allocation)});
        fields.push_back({"first_app_wp", std::to_string(path_size(allocation->firstApproachPath))});
        fields.push_back({"obs_relo_paths", allocation->obsReloPaths ? std::to_string(allocation->obsReloPaths->size()) : "0"});
        fields.push_back({"edge_groups", std::to_string(allocation->paths.size())});
        fields.push_back({"edge_transit_paths", std::to_string(allocation->edgeTransitPaths.size())});
    }
    trace_event("try_result", fields);
}

inline void log_commit(int depth, const FinalAllocation &allocation)
{
    trace_event("commit",
                {{"depth", std::to_string(depth)},
                 {"object", allocation.object.name},
                 {"goal", allocation.goal.name},
                 {"allocation_hash", allocation_hash(allocation)},
                 {"snapshot_mo_raw", map_raw_order(allocation.snapshot.mo_list)},
                 {"snapshot_delivered_raw", map_raw_order(allocation.snapshot.delivered_list)},
                 {"obs_relo_update_raw", state_map_raw_order(allocation.obsReloUpdate)}});
}

inline void log_backtrack(int depth, const FinalAllocation &allocation)
{
    trace_event("backtrack",
                {{"depth", std::to_string(depth)},
                 {"object", allocation.object.name},
                 {"goal", allocation.goal.name},
                 {"allocation_hash", allocation_hash(allocation)}});
}

inline void log_candidate_invalidated(int depth, const LowestCostInfo &candidate)
{
    trace_event("candidate_invalidated",
                {{"depth", std::to_string(depth)},
                 {"object", candidate.objectName},
                 {"goal", candidate.goalName},
                 {"row", std::to_string(candidate.row)},
                 {"col", std::to_string(candidate.col)}});
}

inline std::string final_sequence_stable_hash(
    const std::vector<FinalAllocation> &sequence)
{
    std::ostringstream oss;
    oss << "size=" << sequence.size();
    for (const auto &allocation : sequence)
        oss << "|" << allocation_hash(allocation);
    return hex_u64(fnv1a_string(oss.str()));
}

inline void write_final_sequence_csv(
    const std::vector<FinalAllocation> &sequence)
{
    auto &s = session();
    if (!s.initialized)
        return;

    std::ofstream csv(s.final_csv_path);
    if (!csv.is_open())
        return;

    csv << "task_index,object,goal,cost,row,col,allocation_hash,"
        << "first_app_waypoints,first_app_hash,obs_relo_paths,obs_relo_hash,"
        << "edge_groups,edge_path_hashes,edge_transit_paths,edge_transit_hashes,"
        << "vertex_chain,snapshot_mo_raw_order,snapshot_mo_sorted_hash,"
        << "snapshot_delivered_raw_order,snapshot_delivered_sorted_hash,"
        << "obs_relo_update_raw_order,obs_relo_update_sorted_hash\n";

    csv << std::setprecision(17);
    for (std::size_t i = 0; i < sequence.size(); ++i)
    {
        const auto &allocation = sequence[i];

        std::ostringstream obs_hashes;
        if (allocation.obsReloPaths)
        {
            for (std::size_t j = 0; j < allocation.obsReloPaths->size(); ++j)
            {
                if (j > 0)
                    obs_hashes << ";";
                const auto &path = allocation.obsReloPaths->at(j);
                obs_hashes << j << ":" << (path.is_pushing ? "P" : "T")
                           << ":" << path_hash(path.toStatePath());
            }
        }

        std::ostringstream edge_hashes;
        for (std::size_t j = 0; j < allocation.paths.size(); ++j)
        {
            if (j > 0)
                edge_hashes << ";";
            edge_hashes << j << ":" << edge_data_hash(allocation.paths[j]);
        }

        std::ostringstream transit_hashes;
        for (std::size_t j = 0; j < allocation.edgeTransitPaths.size(); ++j)
        {
            if (j > 0)
                transit_hashes << ";";
            transit_hashes << j << ":" << path_hash(allocation.edgeTransitPaths[j]);
        }

        std::ostringstream vertex_chain;
        for (std::size_t j = 0; j < allocation.vertexChain.size(); ++j)
        {
            if (j > 0)
                vertex_chain << ";";
            const auto &vertex = allocation.vertexChain[j];
            vertex_chain << j << ":" << vertex.name
                         << "[" << vertex.orientationIndex << "]";
        }

        const auto snapshot_mo_sorted = sorted_object_map_string(allocation.snapshot.mo_list);
        const auto snapshot_delivered_sorted = sorted_object_map_string(allocation.snapshot.delivered_list);
        const auto obs_update_sorted = sorted_state_map_string(allocation.obsReloUpdate);

        csv << i << ","
            << csv_escape(allocation.object.name) << ","
            << csv_escape(allocation.goal.name) << ","
            << allocation.cost << ","
            << allocation.row << ","
            << allocation.col << ","
            << allocation_hash(allocation) << ","
            << path_size(allocation.firstApproachPath) << ","
            << path_hash(allocation.firstApproachPath) << ","
            << (allocation.obsReloPaths ? allocation.obsReloPaths->size() : 0) << ","
            << csv_escape(obs_hashes.str()) << ","
            << allocation.paths.size() << ","
            << csv_escape(edge_hashes.str()) << ","
            << allocation.edgeTransitPaths.size() << ","
            << csv_escape(transit_hashes.str()) << ","
            << csv_escape(vertex_chain.str()) << ","
            << csv_escape(map_raw_order(allocation.snapshot.mo_list)) << ","
            << hex_u64(fnv1a_string(snapshot_mo_sorted)) << ","
            << csv_escape(map_raw_order(allocation.snapshot.delivered_list)) << ","
            << hex_u64(fnv1a_string(snapshot_delivered_sorted)) << ","
            << csv_escape(state_map_raw_order(allocation.obsReloUpdate)) << ","
            << hex_u64(fnv1a_string(obs_update_sorted)) << "\n";
    }
}

inline void log_planning_outcome(bool ok,
                                 bool timeout,
                                 long long duration_ms,
                                 double total_path_length,
                                 double total_pushing_length,
                                 int obs_relocations,
                                 int pre_relocations,
                                 const std::vector<FinalAllocation> &sequence)
{
    auto &s = session();
    if (!s.initialized)
        return;

    s.summary << "planning_ok=" << (ok ? 1 : 0) << "\n";
    s.summary << "timeout=" << (timeout ? 1 : 0) << "\n";
    s.summary << "duration_ms=" << duration_ms << "\n";
    s.summary << "total_path_length=" << std::setprecision(17) << total_path_length << "\n";
    s.summary << "total_pushing_length=" << std::setprecision(17) << total_pushing_length << "\n";
    s.summary << "obs_relocations=" << obs_relocations << "\n";
    s.summary << "pre_relocations=" << pre_relocations << "\n";
    s.summary << "final_sequence_size=" << sequence.size() << "\n";
    s.summary << "final_sequence_stable_fnv1a64_at_planning_outcome="
              << final_sequence_stable_hash(sequence) << "\n";
    write_final_sequence_csv(sequence);
    trace_event("planning_outcome",
                {{"ok", ok ? "1" : "0"},
                 {"timeout", timeout ? "1" : "0"},
                 {"duration_ms", std::to_string(duration_ms)},
                 {"final_sequence_size", std::to_string(sequence.size())},
                 {"final_sequence_stable_hash", final_sequence_stable_hash(sequence)}});
}

inline void log_serialized_final_sequence(
    const std::vector<FinalAllocation> &sequence,
    const std::string &binary_data,
    const std::string &base64_data,
    const std::string &output_path)
{
    auto &s = session();
    if (!s.initialized)
        return;

    s.summary << "output_b64_path=" << output_path << "\n";
    s.summary << "serialized_binary_size=" << binary_data.size() << "\n";
    s.summary << "serialized_binary_fnv1a64="
              << hex_u64(fnv1a_string(binary_data)) << "\n";
    s.summary << "serialized_base64_size=" << base64_data.size() << "\n";
    s.summary << "serialized_base64_fnv1a64="
              << hex_u64(fnv1a_string(base64_data)) << "\n";
    s.summary << "final_sequence_stable_fnv1a64_at_serialize="
              << final_sequence_stable_hash(sequence) << "\n";
    s.summary << "final_sequence_csv_represents=serialized_state_after_trajectory_generation\n";
    write_final_sequence_csv(sequence);

    trace_event("serialized_final_sequence",
                {{"output_path", output_path},
                 {"binary_hash", hex_u64(fnv1a_string(binary_data))},
                 {"base64_hash", hex_u64(fnv1a_string(base64_data))},
                 {"stable_hash", final_sequence_stable_hash(sequence)}});
}

inline void finish()
{
    auto &s = session();
    if (!s.initialized)
        return;
    if (s.summary.is_open())
        s.summary.flush();
    if (s.trace.is_open())
        s.trace.flush();
    std::cout << "[ReloPushBossDiag] Wrote reproducibility diagnostics: "
              << s.summary_path << std::endl;
}
} // namespace ReloPushBossDiagnostics

#endif // RELOPUSH_BOSS_DIAGNOSTICS_HPP
