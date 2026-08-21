// ZmqChannels.h
//
// Raw libzmq (zmq.h C API only -- see VescDriver/CMakeLists.txt's HARD
// PORTABILITY RULES; never cppzmq/zmq.hpp anywhere in this folder) wrapper
// around the driver's three ZMQ endpoints, all BOUND by the driver per the
// frozen wire/control-protocol contract:
//   - SUB  tcp://*:<ackermann_port>, subscribed EXACTLY to
//     "/<robot>/ackermann" -- 2 (or more) frames [topic, payload...];
//     draining every pending message each poll and keeping only the
//     newest payload for the subscribed topic (tolerates extra
//     continuation frames by keeping the LAST one, per the contract).
//   - REP  tcp://*:<control_port> -- single-frame plain-JSON
//     request/reply, strict recv-then-send discipline enforced here so a
//     caller bug (replying twice, or polling again before replying) can't
//     desync the REP socket's internal state machine.
//   - PUB  tcp://*:<telemetry_port> -- 2-frame [topic, payload] publishes.
//
// This class is purely I/O plumbing: it hands the caller raw strings
// (topic/payload/request/reply JSON text) and never touches
// AckermannCodec/DriverCore/nlohmann itself, so it stays decoupled from
// those and is trivially usable from a test that wants to talk to a real
// running driver over these same three sockets (see vesc_driver_tests.cpp's
// pty+ZMQ integration test).
//
// Non-blocking throughout (zmq_poll with a 0ms timeout); the caller is
// expected to call poll_ackermann()/poll_control_request() once per control
// loop iteration and to sleep itself between iterations.

#ifndef VESC_DRIVER_ZMQ_CHANNELS_H_
#define VESC_DRIVER_ZMQ_CHANNELS_H_

#include <string>

namespace vesc {

class ZmqChannels {
   public:
    ZmqChannels() = default;
    ~ZmqChannels();

    ZmqChannels(const ZmqChannels&) = delete;
    ZmqChannels& operator=(const ZmqChannels&) = delete;

    // Creates the ZMQ context and binds all three sockets:
    //   SUB  tcp://*:<ackermann_port>  (subscribed to "/<robot_name>/ackermann")
    //   REP  tcp://*:<control_port>
    //   PUB  tcp://*:<telemetry_port>
    // Returns true on success. On any failure, returns false with `error`
    // describing which step failed (via zmq_strerror), and leaves the
    // object in a safely-destructible (shutdown()-equivalent) state -- a
    // caller that gets false back should not call any other method except
    // shutdown()/the destructor.
    bool init(const std::string& robot_name, int ackermann_port, int control_port, int telemetry_port,
               std::string* error);

    // Closes every socket and destroys the context. Idempotent; also
    // called from the destructor, so an explicit call is only needed if the
    // caller wants sockets torn down before the object itself goes out of
    // scope (e.g. to unbind ports for a following test).
    void shutdown();

    // Drains every ackermann message currently queued on the SUB socket
    // (non-blocking) and, if at least one arrived, sets `*payload_out` to
    // the payload of the NEWEST one (per message: keeps the LAST frame of
    // that message, tolerating extra continuation frames past
    // [topic,payload], exactly per the frozen wire contract) and returns
    // true. Returns false (payload_out untouched) if nothing was queued.
    bool poll_ackermann(std::string* payload_out);

    // Non-blocking check for one pending control REP request. Returns true
    // (and sets *request_json_out) at most once per send_control_reply()
    // call -- calling this again before replying to an already-received
    // request is a caller bug and simply returns false every time until
    // send_control_reply() is called, since the REP socket cannot legally
    // recv() a second request before its first is answered.
    bool poll_control_request(std::string* request_json_out);

    // Sends `reply_json` as the single-frame REP reply to the request
    // poll_control_request() most recently returned true for. Returns
    // false (no-op) if no request is currently outstanding.
    bool send_control_reply(const std::string& reply_json);

    // Publishes a 2-frame [topic, payload] message on the telemetry PUB
    // socket. Returns true on success.
    bool publish_telemetry(const std::string& topic, const std::string& payload_json);

    bool is_initialized() const { return ctx_ != nullptr; }

   private:
    void* ctx_ = nullptr;
    void* sub_ = nullptr;
    void* rep_ = nullptr;
    void* pub_ = nullptr;
    std::string ackermann_topic_;
    bool reply_pending_ = false;
};

}  // namespace vesc

#endif  // VESC_DRIVER_ZMQ_CHANNELS_H_
