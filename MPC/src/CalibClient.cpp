#include "CalibClient.h"

#include <nlohmann/json.hpp>

namespace mpc {

// ---------------------------------------------------------------------
// ZmqDriverClient
// ---------------------------------------------------------------------
ZmqDriverClient::ZmqDriverClient(zmq::context_t& ctx, std::string endpoint)
    : ctx_(ctx), endpoint_(std::move(endpoint)) {
    connect();
}

mpc::DriverReply ZmqDriverClient::ping() { return request({{"cmd", "ping"}}); }
mpc::DriverReply ZmqDriverClient::set_source(const std::string& source) {
    return request({{"cmd", "set_source"}, {"source", source}});
}
mpc::DriverReply ZmqDriverClient::raw(mpc::RawMode mode, double value, double ttl_ms) {
    return request({{"cmd", "raw"}, {"mode", mpc::to_string(mode)}, {"value", value}, {"ttl_ms", ttl_ms}});
}
mpc::DriverReply ZmqDriverClient::servo(double value) { return request({{"cmd", "servo"}, {"value", value}}); }
mpc::DriverReply ZmqDriverClient::stop() { return request({{"cmd", "stop"}}); }

void ZmqDriverClient::connect() {
    sock_ = zmq::socket_t(ctx_, zmq::socket_type::req);
    sock_.set(zmq::sockopt::linger, 0);
    sock_.set(zmq::sockopt::rcvtimeo, 500);
    sock_.set(zmq::sockopt::sndtimeo, 500);
    sock_.connect(endpoint_);
}

mpc::DriverReply ZmqDriverClient::request(const nlohmann::json& req_json) {
    mpc::DriverReply out;
    const std::string req_str = req_json.dump();
    for (int attempt = 0; attempt < 2; ++attempt) {
        zmq::message_t msg(req_str.data(), req_str.size());
        auto sent = sock_.send(msg, zmq::send_flags::none);
        if (!sent.has_value()) {
            connect();
            continue;
        }
        zmq::message_t reply;
        auto recvd = sock_.recv(reply, zmq::recv_flags::none);
        if (!recvd.has_value()) {
            // Timed out -- REQ socket state is now desynced; must
            // recreate before the retry (or before any future call).
            connect();
            continue;
        }
        const std::string reply_str(static_cast<char*>(reply.data()), reply.size());
        const nlohmann::json j = nlohmann::json::parse(reply_str, nullptr, false);
        if (j.is_discarded() || !j.is_object()) {
            out.ok = false;
            out.error = "malformed control reply JSON";
            return out;
        }
        out.ok = j.value("ok", false);
        out.error = j.value("error", std::string());
        out.applied_value = j.value("applied_value", 0.0);
        return out;
    }
    out.ok = false;
    out.error = "driver control REQ timed out twice (endpoint " + endpoint_ + " unreachable?)";
    return out;
}

// ---------------------------------------------------------------------
// DriverRestoreGuard
// ---------------------------------------------------------------------
DriverRestoreGuard::DriverRestoreGuard(mpc::IDriverClient& driver) : driver_(driver) {}
DriverRestoreGuard::~DriverRestoreGuard() {
    if (!restored_) {
        try {
            restore();
        } catch (...) {
            // Best-effort only -- nothing left to report to during unwind.
        }
    }
}

bool DriverRestoreGuard::restore() {
    if (restored_) return restore_ok_;
    restored_ = true;
    try {
        driver_.stop();
        const mpc::DriverReply reply = driver_.set_source("ackermann");
        restore_ok_ = reply.ok;
        restore_error_ = reply.error;
    } catch (const std::exception& ex) {
        restore_ok_ = false;
        restore_error_ = ex.what();
    } catch (...) {
        restore_ok_ = false;
        restore_error_ = "unknown exception during driver restore";
    }
    return restore_ok_;
}

// ---------------------------------------------------------------------
// Pose / telemetry wire parsing.
// ---------------------------------------------------------------------
bool parse_pose(const std::string& payload, double self_t, mpc::CalibSample* out) {
    const nlohmann::json j = nlohmann::json::parse(payload, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return false;
    if (!j.contains("x") || !j.contains("y") || !j.contains("yaw")) return false;
    out->t = self_t;
    out->x = j.at("x").get<double>();
    out->y = j.at("y").get<double>();
    out->yaw = j.at("yaw").get<double>();
    return true;
}

mpc::TelemetrySample parse_telemetry(const nlohmann::json& j, double self_t) {
    mpc::TelemetrySample s;
    s.t = self_t;
    s.erpm = j.value("erpm", 0.0);
    s.duty = j.value("duty", 0.0);
    s.current_motor = j.value("current_motor", 0.0);
    s.v_in = j.value("v_in", 0.0);
    return s;
}

// ---------------------------------------------------------------------
// LiveChannels
// ---------------------------------------------------------------------
LiveChannels::LiveChannels(const mpc::MotorCalibConfig& cfg)
    : ctx_(1),
      control_endpoint_("tcp://" + cfg.robot_ip + ":" + std::to_string(cfg.control_port)),
      driver_(ctx_, control_endpoint_) {
    loc_sub_ = zmq::socket_t(ctx_, zmq::socket_type::sub);
    loc_sub_.set(zmq::sockopt::linger, 0);
    loc_sub_.connect(cfg.localization_endpoint);
    loc_sub_.set(zmq::sockopt::subscribe, "/" + cfg.robot_topic_name + "/localization");

    telem_sub_ = zmq::socket_t(ctx_, zmq::socket_type::sub);
    telem_sub_.set(zmq::sockopt::linger, 0);
    const std::string telem_endpoint = "tcp://" + cfg.robot_ip + ":" + std::to_string(cfg.telemetry_port);
    telem_sub_.connect(telem_endpoint);
    telem_sub_.set(zmq::sockopt::subscribe, "/" + cfg.robot_topic_name + "/vesc_telemetry");

    epoch_ = std::chrono::steady_clock::now();
}

double LiveChannels::now_s() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
}

// BURST-DEBOUNCE (pose only): the localization wire payload carries no "t"
// field, so each sample is self-timestamped with THIS process's own now_s()
// at the moment it's drained. If this process's own loop ever falls behind
// the publisher's cadence (OS scheduling jitter -- routine when several
// wrapped subprocesses share a host, as in the e2e test), several already-
// queued messages get drained back-to-back with near-zero WALL time between
// them even though they were PUBLISHED tens of ms apart -- naively self-
// timestamping every one of them would hand the VelocityEstimator an
// implausibly tiny dt for a real dx, producing a velocity spike large enough
// to spuriously trip max_speed_abort_mps. Guard against this by never
// feeding two pose samples closer than kMinPoseFeedGapS apart (by OUR OWN
// receipt-time gap, tracked across pump()/drain_poses() calls, not just
// within one burst) -- under normal (non-bursty) conditions every sample
// clears this trivially, since the publisher's real cadence is far coarser
// than kMinPoseFeedGapS.
mpc::TrialState LiveChannels::pump(mpc::TrialRunner& runner) {
    bool any = false;
    while (true) {
        zmq::message_t topic_msg;
        auto r1 = loc_sub_.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!r1.has_value()) break;
        if (!topic_msg.more()) break;
        zmq::message_t payload_msg;
        auto r2 = loc_sub_.recv(payload_msg, zmq::recv_flags::none);
        if (!r2.has_value()) break;
        const std::string payload(static_cast<char*>(payload_msg.data()), payload_msg.size());
        const double arrival_t = now_s();
        if (arrival_t - last_fed_pose_t_ < kMinPoseFeedGapS) continue;  // burst artifact -- drop, don't feed.
        mpc::CalibSample sample;
        if (parse_pose(payload, arrival_t, &sample)) {
            last_fed_pose_t_ = arrival_t;
            runner.step(sample.t, true, sample, false, {});
            any = true;
        }
    }
    while (true) {
        zmq::message_t topic_msg;
        auto r1 = telem_sub_.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!r1.has_value()) break;
        if (!topic_msg.more()) break;
        zmq::message_t payload_msg;
        auto r2 = telem_sub_.recv(payload_msg, zmq::recv_flags::none);
        if (!r2.has_value()) break;
        const std::string payload(static_cast<char*>(payload_msg.data()), payload_msg.size());
        const nlohmann::json j = nlohmann::json::parse(payload, nullptr, false);
        if (!j.is_discarded() && j.is_object()) {
            const double t = now_s();
            runner.step(t, false, {}, true, parse_telemetry(j, t));
            any = true;
        }
    }
    if (!any) {
        runner.step(now_s(), false, {}, false, {});
    }
    return runner.state();
}

std::vector<mpc::CalibSample> LiveChannels::drain_poses() {
    std::vector<mpc::CalibSample> out;
    while (true) {
        zmq::message_t topic_msg;
        auto r1 = loc_sub_.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!r1.has_value()) break;
        if (!topic_msg.more()) break;
        zmq::message_t payload_msg;
        auto r2 = loc_sub_.recv(payload_msg, zmq::recv_flags::none);
        if (!r2.has_value()) break;
        const std::string payload(static_cast<char*>(payload_msg.data()), payload_msg.size());
        const double arrival_t = now_s();
        if (arrival_t - last_fed_pose_t_ < kMinPoseFeedGapS) continue;  // burst artifact -- drop.
        mpc::CalibSample sample;
        if (parse_pose(payload, arrival_t, &sample)) {
            last_fed_pose_t_ = arrival_t;
            out.push_back(sample);
        }
    }
    return out;
}

std::vector<mpc::TelemetrySample> LiveChannels::drain_telemetry() {
    std::vector<mpc::TelemetrySample> out;
    while (true) {
        zmq::message_t topic_msg;
        auto r1 = telem_sub_.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!r1.has_value()) break;
        if (!topic_msg.more()) break;
        zmq::message_t payload_msg;
        auto r2 = telem_sub_.recv(payload_msg, zmq::recv_flags::none);
        if (!r2.has_value()) break;
        const std::string payload(static_cast<char*>(payload_msg.data()), payload_msg.size());
        const nlohmann::json j = nlohmann::json::parse(payload, nullptr, false);
        if (!j.is_discarded() && j.is_object()) {
            out.push_back(parse_telemetry(j, now_s()));
        }
    }
    return out;
}

}  // namespace mpc
