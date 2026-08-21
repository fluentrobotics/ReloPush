#include "PortDiscovery.h"

#include "SerialPort.h"
#include "VescProtocol.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <thread>

#include <dirent.h>
#include <unistd.h>

namespace vesc {

namespace {

// Case-insensitive substring search.
bool contains_ci(const std::string& haystack, const std::string& needle) {
    std::string h = haystack;
    std::string n = needle;
    std::transform(h.begin(), h.end(), h.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return h.find(n) != std::string::npos;
}

// Higher = more confident this by-id name is the VESC. Checked in the
// documented priority order (ChibiOS > VESC > STMicroelectronics); a by-id
// entry that matches none of them still outranks the ttyACM fallback tier
// (some VESC forks/bootloaders may not embed a recognizable string, but a
// by-id symlink existing at all is still more specific than a bare
// ttyACMn guess).
int score_by_id_name(const std::string& name) {
    static const char* const kKeywordsInPriorityOrder[] = {"ChibiOS", "VESC", "STMicroelectronics"};
    constexpr int kNumKeywords = 3;
    for (int i = 0; i < kNumKeywords; ++i) {
        if (contains_ci(name, kKeywordsInPriorityOrder[i])) {
            return 300 - i;  // 300, 299, 298
        }
    }
    return 100;  // recognized as by-id, but no recognized keyword
}

}  // namespace

std::vector<PortCandidate> rank_candidates(const std::vector<ByIdEntry>& by_id_entries,
                                            const std::vector<std::string>& ttyacm_fallbacks) {
    std::vector<PortCandidate> out;
    out.reserve(by_id_entries.size() + ttyacm_fallbacks.size());

    for (size_t i = 0; i < by_id_entries.size(); ++i) {
        const ByIdEntry& e = by_id_entries[i];
        PortCandidate c;
        c.path = e.resolved_path;
        c.source = "by-id:" + e.name;
        // Multiply the keyword tier so it always dominates the tie-break
        // term below, then subtract input index to keep earlier by-id
        // entries ahead of later ones within the same tier.
        c.score = score_by_id_name(e.name) * 1000 - static_cast<int>(i);
        out.push_back(c);
    }
    for (size_t i = 0; i < ttyacm_fallbacks.size(); ++i) {
        PortCandidate c;
        c.path = ttyacm_fallbacks[i];
        c.source = "ttyACM-fallback";
        // Always below every by-id tier (score_by_id_name()*1000 >= 100000
        // minus a small index term); ttyACM0 ranks above ttyACM1, etc.
        c.score = -static_cast<int>(i);
        out.push_back(c);
    }

    std::stable_sort(out.begin(), out.end(),
                      [](const PortCandidate& a, const PortCandidate& b) { return a.score > b.score; });

    // De-duplicate by resolved path (a by-id symlink commonly resolves to
    // exactly the ttyACMn path that also appears in the fallback list) --
    // keep only the first (highest-ranked) occurrence of each path so the
    // same device is never probed twice.
    std::vector<PortCandidate> deduped;
    deduped.reserve(out.size());
    for (auto& c : out) {
        const bool already_present =
            std::any_of(deduped.begin(), deduped.end(), [&](const PortCandidate& d) { return d.path == c.path; });
        if (!already_present) {
            deduped.push_back(c);
        }
    }
    return deduped;
}

namespace {

std::vector<ByIdEntry> enumerate_by_id() {
    std::vector<ByIdEntry> entries;
    const char* kDir = "/dev/serial/by-id";
    DIR* dir = opendir(kDir);
    if (!dir) {
        return entries;  // no by-id dir (nothing plugged in, or non-udev system) -- not an error
    }
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        const std::string name = ent->d_name;
        if (name == "." || name == "..") {
            continue;
        }
        const std::string full = std::string(kDir) + "/" + name;
        char resolved[4096];
        if (realpath(full.c_str(), resolved) != nullptr) {
            entries.push_back(ByIdEntry{name, std::string(resolved)});
        }
    }
    closedir(dir);
    // readdir() order is not guaranteed -- sort by name for a deterministic
    // candidate order run to run.
    std::sort(entries.begin(), entries.end(),
              [](const ByIdEntry& a, const ByIdEntry& b) { return a.name < b.name; });
    return entries;
}

std::vector<std::string> enumerate_ttyacm_fallbacks() {
    std::vector<std::string> out;
    for (int i = 0; i <= 9; ++i) {
        const std::string path = "/dev/ttyACM" + std::to_string(i);
        if (access(path.c_str(), F_OK) == 0) {
            out.push_back(path);
        }
    }
    return out;
}

// Opens `path` at `baud`, sends a FW_VERSION request, and waits up to
// `timeout_ms` for a valid reply via the streaming frame decoder. Returns
// the reply (ok=false on any failure: can't open, write fails, or no valid
// reply arrives in time).
FwVersionReply probe_fw_version(const std::string& path, int timeout_ms, int baud, std::string* note) {
    FwVersionReply reply;
    SerialPort port;
    if (!port.open(path, baud)) {
        *note = "open failed: " + port.last_error();
        return reply;
    }
    if (!port.write_all(encode_frame(build_fw_version()))) {
        *note = "write failed: " + port.last_error();
        return reply;
    }

    FrameDecoder decoder;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<uint8_t> buf;
        const int n = port.read_available(&buf);
        if (n > 0) {
            decoder.feed(buf);
            std::vector<uint8_t> payload;
            while (decoder.pop_payload(&payload)) {
                FwVersionReply candidate = parse_fw_version(payload);
                if (candidate.ok) {
                    *note = "fw " + std::to_string(candidate.major) + "." + std::to_string(candidate.minor);
                    return candidate;
                }
            }
        } else if (n < 0) {
            *note = "read error: " + port.last_error();
            return reply;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    *note = "no valid FW_VERSION reply within " + std::to_string(timeout_ms) + "ms";
    return reply;
}

}  // namespace

DiscoveryResult find_vesc_port(const std::string& override_path, int probe_timeout_ms, int baud) {
    DiscoveryResult result;

    std::vector<PortCandidate> candidates;
    if (!override_path.empty()) {
        PortCandidate c;
        c.path = override_path;
        c.source = "override";
        c.score = 0;
        candidates.push_back(c);
    } else {
        candidates = rank_candidates(enumerate_by_id(), enumerate_ttyacm_fallbacks());
    }

    for (const auto& c : candidates) {
        std::string note;
        const FwVersionReply reply = probe_fw_version(c.path, probe_timeout_ms, baud, &note);

        DiscoveryLogEntry entry;
        entry.path = c.path;
        entry.source = c.source;
        entry.responded = reply.ok;
        entry.note = note;
        result.log.push_back(entry);

        if (reply.ok) {
            result.ok = true;
            result.path = c.path;
            result.fw_major = reply.major;
            result.fw_minor = reply.minor;
            return result;
        }
    }
    return result;  // ok=false, log has every candidate tried
}

}  // namespace vesc
