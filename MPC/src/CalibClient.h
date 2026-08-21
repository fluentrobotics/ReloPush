#pragma once

// CalibClient.h/.cpp -- the real ZMQ-backed pieces of the motor calibration
// tool's live-hardware wiring, extracted verbatim out of motor_calibration.cpp
// (see that file's own header comment for the FROZEN DRIVER CONTROL PROTOCOL
// context these classes implement against) so a second binary (drive_test)
// can reuse them without depending on motor_calibration.cpp's campaign-runner
// CLI/sweep logic.
//
// Extracted here, unchanged in behavior:
//   - mpc::ZmqDriverClient : mpc::IDriverClient  (REQ/REP to the driver's
//     control port, single-retry-after-resocket on timeout/send failure)
//   - mpc::DriverRestoreGuard                    (RAII: restores "ackermann"
//     source on every exit path once "calib" has been armed)
//   - mpc::LiveChannels                          (owns the ZMQ context, the
//     driver REQ client, and the localization/telemetry SUB sockets; drains
//     them either via TrialRunner::step() -- pump(), unchanged -- or via the
//     new drain_poses()/drain_telemetry() accessors added for drive_test,
//     which need the raw samples without a TrialRunner state machine)
//
// namespace mpc:: is the natural extension of MotorCalibCore.h's own mpc::
// API surface -- callers write mpc::ZmqDriverClient, mpc::LiveChannels,
// mpc::DriverRestoreGuard.

#include "mpc/MotorCalibCore.h"

#include <zmq.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace mpc {

// ---------------------------------------------------------------------
// ZmqDriverClient: REQ/REP client for the driver's control-plane port
// (FROZEN DRIVER CONTROL PROTOCOL). ZMQ REQ sockets enforce strict
// send/recv lockstep -- a timed-out recv() (or a failed send()) leaves the
// socket unable to legally send again without first receiving the missing
// reply, a classic REQ/REP lockstep hazard whose only fix is to recreate
// the socket. request() therefore recreates + retries EXACTLY ONCE before
// giving up (ok=false, error describes the timeout).
// ---------------------------------------------------------------------
class ZmqDriverClient : public mpc::IDriverClient {
public:
    ZmqDriverClient(zmq::context_t& ctx, std::string endpoint);

    mpc::DriverReply ping() override;
    mpc::DriverReply set_source(const std::string& source) override;
    mpc::DriverReply raw(mpc::RawMode mode, double value, double ttl_ms) override;
    mpc::DriverReply servo(double value) override;
    mpc::DriverReply stop() override;

private:
    void connect();
    mpc::DriverReply request(const nlohmann::json& req_json);

    zmq::context_t& ctx_;
    std::string endpoint_;
    zmq::socket_t sock_;
};

// ---------------------------------------------------------------------
// RAII guard: makes sure the driver is put back on "ackermann" source
// exactly once on EVERY exit path once set_source("calib") has actually
// been issued -- normal completion, a mid-campaign abort, Ctrl-C, or an
// exception unwinding out of the caller's try block. Without this, an
// exception thrown after set_source(calib) would terminate the process
// with the driver stuck in calib source (accepting raw duty/erpm/current
// commands, NOT the ackermann stream) until something else resets it.
//
// restore() is idempotent and may be called explicitly (so the normal
// path can capture the DriverReply for reporting); if it never was, the
// destructor calls it itself as a last-resort fallback -- any error at
// that point is swallowed (nothing left to report it to during unwind).
// ---------------------------------------------------------------------
class DriverRestoreGuard {
public:
    explicit DriverRestoreGuard(mpc::IDriverClient& driver);
    ~DriverRestoreGuard();

    DriverRestoreGuard(const DriverRestoreGuard&) = delete;
    DriverRestoreGuard& operator=(const DriverRestoreGuard&) = delete;

    bool restore();

    const std::string& error() const { return restore_error_; }

private:
    mpc::IDriverClient& driver_;
    bool restored_ = false;
    bool restore_ok_ = false;
    std::string restore_error_;
};

// ---------------------------------------------------------------------
// Pose / telemetry wire parsing (used internally by LiveChannels).
// ---------------------------------------------------------------------
bool parse_pose(const std::string& payload, double self_t, mpc::CalibSample* out);
mpc::TelemetrySample parse_telemetry(const nlohmann::json& j, double self_t);

// ---------------------------------------------------------------------
// LiveChannels: the three real ZMQ endpoints + a self-timestamping
// drain-keep-all pump that feeds every distinct pose/telemetry sample to a
// mpc::TrialRunner individually (never collapsed to "newest wins" --
// velocity estimation needs every consecutive sample, unlike the ackermann
// SUB's newest-wins policy elsewhere in this repo). drain_poses()/
// drain_telemetry() offer the same drain semantics WITHOUT a TrialRunner,
// for callers (drive_test) that implement their own simpler loop.
// ---------------------------------------------------------------------
class LiveChannels {
public:
    explicit LiveChannels(const mpc::MotorCalibConfig& cfg);

    mpc::IDriverClient& driver() { return driver_; }
    double now_s() const;

    // Drains everything currently pending on both SUB sockets (non-blocking)
    // into `runner` and returns its resulting state(). If nothing was
    // pending at all, still ticks the state machine once with
    // have_pose=false/have_telemetry=false so staleness/timeout checks keep
    // progressing in real time. See CalibClient.cpp for the full
    // BURST-DEBOUNCE doc comment (pose only).
    mpc::TrialState pump(mpc::TrialRunner& runner);

    // Same non-blocking drain + burst-debounce + self-timestamp logic as the
    // pose half of pump(), but appends each parsed sample to the returned
    // vector instead of calling runner.step(). Shares the SAME
    // last_fed_pose_t_ state as pump() -- a caller can use pump() and
    // drain_poses() on the same instance without them interfering.
    std::vector<mpc::CalibSample> drain_poses();

    // Same non-blocking drain + self-timestamp logic as the telemetry half
    // of pump(), returned as a vector.
    std::vector<mpc::TelemetrySample> drain_telemetry();

private:
    static constexpr double kMinPoseFeedGapS = 0.01;  // see pump()'s BURST-DEBOUNCE doc comment.

    zmq::context_t ctx_;
    std::string control_endpoint_;
    ZmqDriverClient driver_;
    zmq::socket_t loc_sub_;
    zmq::socket_t telem_sub_;
    std::chrono::steady_clock::time_point epoch_;
    double last_fed_pose_t_ = -1.0e18;
};

}  // namespace mpc
