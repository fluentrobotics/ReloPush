#include "ZmqChannels.h"

#include <cstring>

#include <zmq.h>

namespace vesc {

namespace {

bool recv_frame_nonblock(void* sock, std::string* out) {
    zmq_msg_t msg;
    zmq_msg_init(&msg);
    const int rc = zmq_msg_recv(&msg, sock, ZMQ_DONTWAIT);
    if (rc < 0) {
        zmq_msg_close(&msg);
        return false;
    }
    out->assign(static_cast<const char*>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    return true;
}

bool socket_has_more(void* sock) {
    int more = 0;
    size_t more_size = sizeof(more);
    if (zmq_getsockopt(sock, ZMQ_RCVMORE, &more, &more_size) != 0) {
        return false;
    }
    return more != 0;
}

bool send_frame(void* sock, const std::string& data, int flags) {
    zmq_msg_t msg;
    zmq_msg_init_size(&msg, data.size());
    if (!data.empty()) {
        std::memcpy(zmq_msg_data(&msg), data.data(), data.size());
    }
    const int rc = zmq_msg_send(&msg, sock, flags);
    if (rc < 0) {
        zmq_msg_close(&msg);
        return false;
    }
    return true;
}

bool poll_readable(void* sock) {
    zmq_pollitem_t item;
    item.socket = sock;
    item.fd = 0;
    item.events = ZMQ_POLLIN;
    item.revents = 0;
    const int rc = zmq_poll(&item, 1, 0);  // 0ms timeout -- purely non-blocking check.
    return rc > 0 && (item.revents & ZMQ_POLLIN) != 0;
}

void* bind_socket(void* ctx, int type, int port, std::string* error, const char* label) {
    void* sock = zmq_socket(ctx, type);
    if (!sock) {
        *error = std::string(label) + ": zmq_socket() failed: " + zmq_strerror(zmq_errno());
        return nullptr;
    }
    const int linger_ms = 0;
    zmq_setsockopt(sock, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));

    const std::string endpoint = "tcp://*:" + std::to_string(port);
    if (zmq_bind(sock, endpoint.c_str()) != 0) {
        *error = std::string(label) + ": zmq_bind('" + endpoint + "') failed: " + zmq_strerror(zmq_errno());
        zmq_close(sock);
        return nullptr;
    }
    return sock;
}

}  // namespace

ZmqChannels::~ZmqChannels() { shutdown(); }

bool ZmqChannels::init(const std::string& robot_name, int ackermann_port, int control_port, int telemetry_port,
                         std::string* error) {
    shutdown();  // idempotent -- guards against a caller re-calling init() without shutting down first.

    ctx_ = zmq_ctx_new();
    if (!ctx_) {
        if (error) *error = std::string("zmq_ctx_new() failed: ") + zmq_strerror(zmq_errno());
        return false;
    }

    std::string err;
    sub_ = bind_socket(ctx_, ZMQ_SUB, ackermann_port, &err, "ackermann SUB");
    if (!sub_) {
        if (error) *error = err;
        shutdown();
        return false;
    }
    ackermann_topic_ = "/" + robot_name + "/ackermann";
    if (zmq_setsockopt(sub_, ZMQ_SUBSCRIBE, ackermann_topic_.c_str(), ackermann_topic_.size()) != 0) {
        if (error) *error = std::string("ZMQ_SUBSCRIBE failed: ") + zmq_strerror(zmq_errno());
        shutdown();
        return false;
    }

    rep_ = bind_socket(ctx_, ZMQ_REP, control_port, &err, "control REP");
    if (!rep_) {
        if (error) *error = err;
        shutdown();
        return false;
    }

    pub_ = bind_socket(ctx_, ZMQ_PUB, telemetry_port, &err, "telemetry PUB");
    if (!pub_) {
        if (error) *error = err;
        shutdown();
        return false;
    }

    reply_pending_ = false;
    return true;
}

void ZmqChannels::shutdown() {
    if (sub_) {
        zmq_close(sub_);
        sub_ = nullptr;
    }
    if (rep_) {
        zmq_close(rep_);
        rep_ = nullptr;
    }
    if (pub_) {
        zmq_close(pub_);
        pub_ = nullptr;
    }
    if (ctx_) {
        zmq_ctx_destroy(ctx_);
        ctx_ = nullptr;
    }
    reply_pending_ = false;
}

bool ZmqChannels::poll_ackermann(std::string* payload_out) {
    if (!sub_) return false;

    bool got = false;
    while (poll_readable(sub_)) {
        std::string topic;
        if (!recv_frame_nonblock(sub_, &topic)) break;

        // Drain every continuation frame, keeping only the LAST as the
        // payload (per contract: "tolerate extra continuation frames by
        // keeping the LAST").
        std::string payload;
        while (socket_has_more(sub_)) {
            std::string frame;
            if (!recv_frame_nonblock(sub_, &frame)) break;
            payload = frame;
        }

        if (topic == ackermann_topic_) {
            *payload_out = payload;
            got = true;
        }
        // Loop again: keep draining so only the OVERALL newest queued
        // message (across this whole poll call) is kept, not just the
        // newest frame of the first message found.
    }
    return got;
}

bool ZmqChannels::poll_control_request(std::string* request_json_out) {
    if (!rep_ || reply_pending_) return false;
    if (!poll_readable(rep_)) return false;

    std::string request;
    if (!recv_frame_nonblock(rep_, &request)) return false;
    // Contract is single-frame; defensively drain (and discard) any
    // unexpected continuation frames so the REP socket's own state machine
    // is never left mid-message.
    while (socket_has_more(rep_)) {
        std::string discard;
        if (!recv_frame_nonblock(rep_, &discard)) break;
    }

    *request_json_out = request;
    reply_pending_ = true;
    return true;
}

bool ZmqChannels::send_control_reply(const std::string& reply_json) {
    if (!rep_ || !reply_pending_) return false;
    const bool ok = send_frame(rep_, reply_json, 0);
    reply_pending_ = false;
    return ok;
}

bool ZmqChannels::publish_telemetry(const std::string& topic, const std::string& payload_json) {
    if (!pub_) return false;
    if (!send_frame(pub_, topic, ZMQ_SNDMORE)) return false;
    return send_frame(pub_, payload_json, 0);
}

}  // namespace vesc
