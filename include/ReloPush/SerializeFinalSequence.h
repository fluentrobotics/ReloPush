#ifndef SERIALIZEFINALSEQUENCE_H
#define SERIALIZEFINALSEQUENCE_H

#include <sstream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <cstddef>  // for std::size_t
#include <cstdint>  // for std::int64_t, std::uint64_t
#include <limits>   // for numeric_limits
#include <stdexcept> // for runtime_error
#include <type_traits> // for enable_if, is_enum
#include <variant>  // for std::variant

// Project headers
#include <GraphData.hpp>
#include <ObjectInfo.hpp>
#include <State.h>
#include <TaskAllocation.hpp>
#include <PlanningContext.hpp>
#include <PathPlanningTools.h>
#include "PlanResult.hpp"
#include <DubinsTools.h>



inline void dump_remaining(std::istream& is) {
    std::streampos current_pos = is.tellg();
    is.seekg(0, is.end);
    std::streampos end_pos = is.tellg();
    is.seekg(current_pos);  // Reset to original position

    std::streamsize remaining_size = end_pos - current_pos;
    std::cout << "Remaining bytes: " << remaining_size << std::endl;

    /*
    if (remaining_size > 0) {
        std::string buffer(remaining_size, '\0');
        is.read(&buffer[0], remaining_size);

        std::cout << "Hex dump of remaining data: ";
        for (unsigned char c : buffer) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
        }
        std::cout << std::dec << std::endl;  // Reset to decimal

        // Reset position again for continued deserialization
        is.seekg(current_pos);
    } else {
        std::cout << "No remaining data." << std::endl;
    }
        */
}




namespace Serialization {

// Declarations for serialize
void serialize(std::ostream& os, int value);
void serialize(std::ostream& os, double value);
void serialize(std::ostream& os, float value);
void serialize(std::ostream& os, bool value);
void serialize(std::ostream& os, std::size_t value);
void serialize(std::ostream& os, std::int64_t value);
void serialize(std::ostream& os, const std::string& value);

void serialize(std::ostream& os, VertexType value);
void serialize(std::ostream& os, ConnectionMode value);
void serialize(std::ostream& os, StateValidity value);

void serialize(std::ostream& os, const ReloPush::State& s);
void serialize(std::ostream& os, const ObjectInfo& oi);
void serialize(std::ostream& os, const VertexData& vd);
void serialize(std::ostream& os, const PreRelocationInfo& pri);
void serialize(std::ostream& os, const EdgePath& ep);
void serialize(std::ostream& os, const EdgeData& ed);
void serialize(std::ostream& os, const WorkspaceBoundary& wb);
void serialize(std::ostream& os, const TurningRadiusPair& trp);
void serialize(std::ostream& os, const PlanningParameters& pp);
void serialize(std::ostream& os, const PlanningContext& pc);
void serialize(std::ostream& os, const FinalAllocation& fa);

void serialize(std::ostream& os, const ObjectMap& value);

// Template declarations for serialize
template<typename T>
void serialize(std::ostream& os, const std::vector<T>& value);

template<typename T>
void serialize(std::ostream& os, const std::shared_ptr<T>& value);

template<typename V>
void serialize(std::ostream& os, const std::unordered_map<std::string, V>& value);

// Declarations for deserialize
void deserialize(std::istream& is, int& value);
void deserialize(std::istream& is, double& value);
void deserialize(std::istream& is, float& value);
void deserialize(std::istream& is, bool& value);
void deserialize(std::istream& is, std::size_t& value);
void deserialize(std::istream& is, std::int64_t& value);
void deserialize(std::istream& is, std::string& value);

void deserialize(std::istream& is, VertexType& value);
void deserialize(std::istream& is, ConnectionMode& value);
void deserialize(std::istream& is, StateValidity& value);

void deserialize(std::istream& is, ReloPush::State& s);
void deserialize(std::istream& is, ObjectInfo& oi);
void deserialize(std::istream& is, VertexData& vd);
void deserialize(std::istream& is, PreRelocationInfo& pri);
void deserialize(std::istream& is, EdgePath& ep);
void deserialize(std::istream& is, EdgeData& ed);
void deserialize(std::istream& is, WorkspaceBoundary& wb);
void deserialize(std::istream& is, TurningRadiusPair& trp);
void deserialize(std::istream& is, PlanningParameters& pp);
void deserialize(std::istream& is, PlanningContext& pc);
void deserialize(std::istream& is, FinalAllocation& fa);

void deserialize(std::istream& is, ObjectMap& value);

// Template declarations for deserialize
template<typename T>
void deserialize(std::istream& is, std::vector<T>& value);

template<typename T>
void deserialize(std::istream& is, std::shared_ptr<T>& value);

template<typename V>
void deserialize(std::istream& is, std::unordered_map<std::string, V>& value);

// Implementations for serialize
inline void serialize(std::ostream& os, int value) {
    os.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

inline void serialize(std::ostream& os, double value) {
    os.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

inline void serialize(std::ostream& os, float value) {
    os.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

inline void serialize(std::ostream& os, bool value) {
    char v = value ? 1 : 0;
    os.write(&v, 1);
}

inline void serialize(std::ostream& os, std::size_t value) {
    std::uint64_t v = static_cast<std::uint64_t>(value);
    os.write(reinterpret_cast<const char*>(&v), sizeof(v));
}

inline void serialize(std::ostream& os, std::int64_t value) {
    os.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

inline void serialize(std::ostream& os, const std::string& value) {
    std::size_t len = value.size();
    serialize(os, len);
    os.write(value.data(), static_cast<std::streamsize>(len));
}

inline void serialize(std::ostream& os, VertexType value) {
    serialize(os, static_cast<int>(value));
}

inline void serialize(std::ostream& os, ConnectionMode value) {
    serialize(os, static_cast<int>(value));
}

inline void serialize(std::ostream& os, StateValidity value) {
    serialize(os, static_cast<int>(value));
}

inline void serialize(std::ostream& os, const ReloPush::State& s) {
    serialize(os, s.x);
    serialize(os, s.y);
    serialize(os, s.yaw);
    serialize(os, s.time);
    serialize(os, s.vel);
    serialize(os, s.is_pushing);
}

inline void serialize(std::ostream& os, const ObjectInfo& oi) {
    serialize(os, oi.name);
    serialize(os, oi.x);
    serialize(os, oi.y);
    serialize(os, oi.nominalOrientation);
    serialize(os, oi.numberOfSides);
    serialize(os, oi.enclosingRadius);
}

inline void serialize(std::ostream& os, const VertexData& vd) {
    serialize(os, vd.type);
    serialize(os, vd.name);
    serialize(os, vd.orientationIndex);
    serialize(os, vd.nominalOrientation);
    serialize(os, vd.x);
    serialize(os, vd.y);
    serialize(os, vd.numberOfSides);
    serialize(os, vd.radius);
}

inline void serialize(std::ostream& os, const PreRelocationInfo& pri) {
    serialize(os, pri.used);
    serialize(os, pri.xRelocated_robot);
    serialize(os, pri.yRelocated_robot);
    serialize(os, pri.yawReloacted_robot);  // Keep original typo
    serialize(os, pri.xRelocated_object);
    serialize(os, pri.yRelocated_object);
    serialize(os, pri.yawRelocated_object);
    serialize(os, pri.extraCost);
    serialize(os, pri.relocatingIndex);
    serialize(os, pri.reason);
}

inline void serialize(std::ostream& os, const EdgePath& ep) {
    serialize(os, ep.is_pushing);
    auto statePath = ep.toStatePath();
    serialize(os, *statePath);
}

inline void serialize(std::ostream& os, const EdgeData& ed) {
    serialize(os, ed.weight);
    serialize(os, ed.srcVertexData);
    serialize(os, ed.sinkVertexData);
    serialize(os, ed.mode);
    serialize(os, ed.preRelo);
    serialize(os, ed.paths);
}

inline void serialize(std::ostream& os, const WorkspaceBoundary& wb) {
    serialize(os, wb.xMin);
    serialize(os, wb.yMin);
    serialize(os, wb.xMax);
    serialize(os, wb.yMax);
}

inline void serialize(std::ostream& os, const TurningRadiusPair& trp) {
    serialize(os, trp.push);
    serialize(os, trp.non_push);
}

inline void serialize(std::ostream& os, const PlanningParameters& pp) {
    serialize(os, pp.boundary);
    serialize(os, pp.turning_rad_pair);
    serialize(os, pp.map_resolution);
    serialize(os, pp.car_width);
    serialize(os, pp.obs_rad);
    serialize(os, pp.LF_push);
    serialize(os, pp.LF_nonpush);
    serialize(os, pp.LB);
    serialize(os, pp.PrePush_dist);
}

inline void serialize(std::ostream& os, const PlanningContext& pc) {
    serialize(os, pc.parameters);
    serialize(os, pc.mo_list);
    serialize(os, pc.delivered_list);
    serialize(os, pc.sample_N);
    serialize(os, pc.timeout_ms);
    serialize(os, pc.print_res);
    serialize(os, pc.use_prelo_optimization);
    serialize(os, pc.num_of_obj);
    serialize(os, pc.no_init_guess);
    serialize(os, pc.sampledPositions);
}

inline void serialize(std::ostream& os, const FinalAllocation& fa) {
    serialize(os, fa.object);
    serialize(os, fa.goal);
    serialize(os, fa.cost);
    serialize(os, fa.row);
    serialize(os, fa.col);
    serialize(os, fa.vertexChain);
    serialize(os, fa.startPose);
    serialize(os, fa.goalPose);
    serialize(os, fa.snapshot);
    serialize(os, fa.paths);
    serialize(os, fa.edgeTransitPaths);
    serialize(os, fa.firstApproachPath);
    serialize(os, fa.obsReloPaths);
    serialize(os, fa.obsReloUpdate);
}

inline void serialize(std::ostream& os, const ObjectMap& value) {
    serialize(os, static_cast<const std::unordered_map<std::string, ObjectInfo>&>(value));
}

// Template implementations for serialize
template<typename T>
void serialize(std::ostream& os, const std::vector<T>& value) {
    std::size_t len = value.size();
    serialize(os, len);
    for (const auto& item : value) {
        serialize(os, item);
    }
}

template<typename T>
void serialize(std::ostream& os, const std::shared_ptr<T>& value) {
    bool is_null = !value;
    serialize(os, is_null);
    if (!is_null) {
        serialize(os, *value);
    }
}

template<typename V>
void serialize(std::ostream& os, const std::unordered_map<std::string, V>& value) {
    std::size_t len = value.size();
    serialize(os, len);
    for (const auto& pair : value) {
        serialize(os, pair.first);
        serialize(os, pair.second);
    }
}

// Deserialization implementations
inline void deserialize(std::istream& is, int& value) {
    is.read(reinterpret_cast<char*>(&value), sizeof(value));
}

inline void deserialize(std::istream& is, double& value) {
    is.read(reinterpret_cast<char*>(&value), sizeof(value));
}

inline void deserialize(std::istream& is, float& value) {
    is.read(reinterpret_cast<char*>(&value), sizeof(value));
}

inline void deserialize(std::istream& is, bool& value) {
    char v;
    is.read(&v, 1);
    value = (v != 0);
}

inline void deserialize(std::istream& is, std::size_t& value) {
    std::uint64_t v;
    is.read(reinterpret_cast<char*>(&v), sizeof(v));
    if (v > std::numeric_limits<std::size_t>::max()) {
        throw std::runtime_error("Deserialized size_t too large");
    }
    value = static_cast<std::size_t>(v);
}

inline void deserialize(std::istream& is, std::int64_t& value) {
    is.read(reinterpret_cast<char*>(&value), sizeof(value));
}

inline void deserialize(std::istream& is, std::string& value) {
    std::size_t len;
    deserialize(is, len);
    value.resize(len);
    is.read(&value[0], static_cast<std::streamsize>(len));
}

inline void deserialize(std::istream& is, VertexType& value) {
    int v;
    deserialize(is, v);
    value = static_cast<VertexType>(v);
}

inline void deserialize(std::istream& is, ConnectionMode& value) {
    int v;
    deserialize(is, v);
    value = static_cast<ConnectionMode>(v);
}

inline void deserialize(std::istream& is, StateValidity& value) {
    int v;
    deserialize(is, v);
    value = static_cast<StateValidity>(v);
}

inline void deserialize(std::istream& is, ReloPush::State& s) {
    deserialize(is, s.x);
    deserialize(is, s.y);
    deserialize(is, s.yaw);
    deserialize(is, s.time);
    deserialize(is, s.vel);
    deserialize(is, s.is_pushing);
}

inline void deserialize(std::istream& is, ObjectInfo& oi) {
    deserialize(is, oi.name);
    deserialize(is, oi.x);
    deserialize(is, oi.y);
    deserialize(is, oi.nominalOrientation);
    deserialize(is, oi.numberOfSides);
    deserialize(is, oi.enclosingRadius);
}

inline void deserialize(std::istream& is, VertexData& vd) {
    deserialize(is, vd.type);
    deserialize(is, vd.name);
    deserialize(is, vd.orientationIndex);
    deserialize(is, vd.nominalOrientation);
    deserialize(is, vd.x);
    deserialize(is, vd.y);
    deserialize(is, vd.numberOfSides);
    deserialize(is, vd.radius);
}

inline void deserialize(std::istream& is, PreRelocationInfo& pri) {
    deserialize(is, pri.used);
    deserialize(is, pri.xRelocated_robot);
    deserialize(is, pri.yRelocated_robot);
    deserialize(is, pri.yawReloacted_robot);
    deserialize(is, pri.xRelocated_object);
    deserialize(is, pri.yRelocated_object);
    deserialize(is, pri.yawRelocated_object);
    deserialize(is, pri.extraCost);
    deserialize(is, pri.relocatingIndex);
    deserialize(is, pri.reason);
}

inline void deserialize(std::istream& is, EdgePath& ep) {
    deserialize(is, ep.is_pushing);
    ReloPush::StatePath sp;
    deserialize(is, sp);
    ep.path = std::make_shared<ReloPush::StatePath>(sp);
}

inline void deserialize(std::istream& is, EdgeData& ed) {
    deserialize(is, ed.weight);
    deserialize(is, ed.srcVertexData);
    deserialize(is, ed.sinkVertexData);
    deserialize(is, ed.mode);
    deserialize(is, ed.preRelo);
    deserialize(is, ed.paths);
}

inline void deserialize(std::istream& is, WorkspaceBoundary& wb) {
    deserialize(is, wb.xMin);
    deserialize(is, wb.yMin);
    deserialize(is, wb.xMax);
    deserialize(is, wb.yMax);
}

inline void deserialize(std::istream& is, TurningRadiusPair& trp) {
    deserialize(is, trp.push);
    deserialize(is, trp.non_push);
}

inline void deserialize(std::istream& is, PlanningParameters& pp) {
    deserialize(is, pp.boundary);
    deserialize(is, pp.turning_rad_pair);
    deserialize(is, pp.map_resolution);
    deserialize(is, pp.car_width);
    deserialize(is, pp.obs_rad);
    deserialize(is, pp.LF_push);
    deserialize(is, pp.LF_nonpush);
    deserialize(is, pp.LB);
    deserialize(is, pp.PrePush_dist);
}

inline void deserialize(std::istream& is, PlanningContext& pc) {
    deserialize(is, pc.parameters);
    deserialize(is, pc.mo_list);
    deserialize(is, pc.delivered_list);
    deserialize(is, pc.sample_N);
    deserialize(is, pc.timeout_ms);
    deserialize(is, pc.print_res);
    deserialize(is, pc.use_prelo_optimization);
    deserialize(is, pc.num_of_obj);
    deserialize(is, pc.no_init_guess);
    deserialize(is, pc.sampledPositions);
    pc.updateObs(pc.mo_list, pc.delivered_list);
}

inline void deserialize(std::istream& is, FinalAllocation& fa) {
    deserialize(is, fa.object);
    deserialize(is, fa.goal);
    deserialize(is, fa.cost);
    deserialize(is, fa.row);
    deserialize(is, fa.col);
    deserialize(is, fa.vertexChain);
    deserialize(is, fa.startPose);
    deserialize(is, fa.goalPose);
    deserialize(is, fa.snapshot);
    deserialize(is, fa.paths);
    deserialize(is, fa.edgeTransitPaths);
    deserialize(is, fa.firstApproachPath);
    deserialize(is, fa.obsReloPaths);
    deserialize(is, fa.obsReloUpdate);
}

// For ObjectMap
inline void deserialize(std::istream& is, ObjectMap& value) {
    std::unordered_map<std::string, ObjectInfo> temp;
    deserialize(is, temp);
    value = ObjectMap(temp);
}

// Template implementations for deserialize
template<typename T>
void deserialize(std::istream& is, std::vector<T>& value) {
    std::size_t len;
    deserialize(is, len);
    value.resize(len);
    for (auto& item : value) {
        deserialize(is, item);
    }
}

template<typename T>
void deserialize(std::istream& is, std::shared_ptr<T>& value) {
    bool is_null;
    deserialize(is, is_null);
    if (is_null) {
        value = nullptr;
    } else {
        value = std::make_shared<T>();
        deserialize(is, *value);
    }
}

template<typename V>
void deserialize(std::istream& is, std::unordered_map<std::string, V>& value) {
    std::size_t len;
    deserialize(is, len);
    value.clear();
    for (std::size_t i = 0; i < len; ++i) {
        std::string key;
        V val;
        deserialize(is, key);
        deserialize(is, val);
        value[key] = val;
    }
}

}  // namespace Serialization

// Global functions
inline std::string serializeFinalSequence(const std::vector<FinalAllocation>& fs) {
    std::ostringstream oss(std::ios::binary);
    std::size_t size = fs.size();
    Serialization::serialize(oss, size);
    for (const auto& fa : fs) {
        Serialization::serialize(oss, fa);
    }
    return oss.str();
}

inline std::vector<FinalAllocation> deserializeFinalSequence(const std::string& data) {
    std::istringstream iss(data, std::ios::binary);
    std::size_t size;
    Serialization::deserialize(iss, size);
    std::vector<FinalAllocation> fs(size);
    for (auto& fa : fs) {
        Serialization::deserialize(iss, fa);
    }
    return fs;
}
#endif
