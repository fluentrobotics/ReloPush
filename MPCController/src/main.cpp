#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>
#include <memory>
#include <atomic>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <tuple>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <ceres/ceres.h>

#include <ReloPush/trajectory.hpp>
#include <ReloPush/base64.h>

// Struct for robot pose/state
struct PoseState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// Ceres cost functor for Ackermann trajectory tracking
struct MPCCostFunctor {
    double x0;
    double y0;
    double yaw0;
    double L;
    double dt;
    int T;

    double w_dist;
    double w_yaw;
    double w_vel;
    double w_lat;
    double w_control;
    double w_delta_rate;

    struct RefState {
        double x;
        double y;
        double yaw;
        double ref_vel;
    };
    std::vector<RefState> ref_states;

    MPCCostFunctor(double x0, double y0, double yaw0, double L, double dt, int T,
                   const std::vector<RefState>& ref_states,
                   double w_dist, double w_yaw, double w_vel, double w_lat, double w_control, double w_delta_rate)
        : x0(x0), y0(y0), yaw0(yaw0), L(L), dt(dt), T(T), ref_states(ref_states),
          w_dist(w_dist), w_yaw(w_yaw), w_vel(w_vel), w_lat(w_lat), w_control(w_control), w_delta_rate(w_delta_rate) {}

    template <typename T_num>
    bool operator()(const T_num* const u, T_num* residuals) const {
        T_num x = T_num(x0);
        T_num y = T_num(y0);
        T_num yaw = T_num(yaw0);

        int res_idx = 0;

        for (int t = 0; t < T; ++t) {
            T_num v = u[2 * t];
            T_num delta = u[2 * t + 1];

            // Kinematic Ackermann update
            x += v * ceres::cos(yaw) * dt;
            y += v * ceres::sin(yaw) * dt;
            yaw += (v / L) * ceres::tan(delta) * dt;

            const auto& ref = ref_states[t];

            // 1. Position error residuals (x and y)
            residuals[res_idx++] = T_num(std::sqrt(w_dist)) * (x - T_num(ref.x));
            residuals[res_idx++] = T_num(std::sqrt(w_dist)) * (y - T_num(ref.y));

            // 2. Heading error residual (shortest angle wrap)
            T_num dyaw = yaw - T_num(ref.yaw);
            T_num wrapped_dyaw = ceres::atan2(ceres::sin(dyaw), ceres::cos(dyaw));
            residuals[res_idx++] = T_num(std::sqrt(w_yaw)) * wrapped_dyaw;

            // 3. Velocity error residual
            residuals[res_idx++] = T_num(std::sqrt(w_vel)) * (v - T_num(ref.ref_vel));

            // 4. Lateral error residual
            T_num dx = T_num(ref.x) - x;
            T_num dy = T_num(ref.y) - y;
            T_num e_y = -ceres::sin(yaw) * dx + ceres::cos(yaw) * dy;
            residuals[res_idx++] = T_num(std::sqrt(w_lat)) * e_y;

            // 5. Control effort residuals
            residuals[res_idx++] = T_num(std::sqrt(w_control)) * v;
            residuals[res_idx++] = T_num(std::sqrt(w_control)) * delta;
        }

        // 6. Steering rate change residuals
        for (int t = 1; t < T; ++t) {
            T_num delta_rate = u[2 * t + 1] - u[2 * (t - 1) + 1];
            residuals[res_idx++] = T_num(std::sqrt(w_delta_rate)) * delta_rate;
        }

        return true;
    }
};

// Base64 helper matching vesc_zmq_sender
std::string encodeAscii(double value) {
    std::ostringstream oss;
    oss << std::setprecision(16) << value;
    return base64_encode(oss.str());
}

// Linear interpolation for reference trajectory waypoints
std::tuple<double, double, double, double, bool> get_ref_state_at_time(
    double abs_time, double traj_start_time, const ReloPush::trajectory& ref_traj) {
    
    const auto& points = *(ref_traj.trajectory_points);
    if (points.empty()) {
        return {0.0, 0.0, 0.0, 0.0, false};
    }
    
    double rel_time = abs_time - traj_start_time;
    
    if (rel_time <= points.front().time) {
        return {points.front().x, points.front().y, points.front().yaw, points.front().ref_vel, points.front().is_pushing};
    }
    if (rel_time >= points.back().time) {
        return {points.back().x, points.back().y, points.back().yaw, points.back().ref_vel, points.back().is_pushing};
    }
    
    auto it = std::lower_bound(points.begin(), points.end(), rel_time, 
        [](const ReloPush::trajectory_elem& elem, double val) {
            return elem.time < val;
        });
        
    int idx = std::distance(points.begin(), it);
    if (idx == 0) {
        return {points.front().x, points.front().y, points.front().yaw, points.front().ref_vel, points.front().is_pushing};
    }
    
    const auto& p0 = points[idx - 1];
    const auto& p1 = points[idx];
    
    double t0 = p0.time;
    double t1 = p1.time;
    double f = (rel_time - t0) / (t1 - t0);
    
    double x = (1.0 - f) * p0.x + f * p1.x;
    double y = (1.0 - f) * p0.y + f * p1.y;
    double ref_vel = (1.0 - f) * p0.ref_vel + f * p1.ref_vel;
    
    double yaw0 = p0.yaw;
    double yaw1 = p1.yaw;
    double dyaw = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
    double yaw = yaw0 + f * dyaw;
    
    return {x, y, yaw, ref_vel, p1.is_pushing};
}

int main(int argc, char** argv) {
    std::string robot_name = "robot1";
    int handshake_port = 11111;
    std::string vesc_endpoint = "tcp://127.0.0.1:3161";
    std::string localization_endpoint = "tcp://127.0.0.1:3261";

    // Parse options
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--robot" && i + 1 < argc) {
            robot_name = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            handshake_port = std::stoi(argv[++i]);
        } else if (arg == "--vesc-endpoint" && i + 1 < argc) {
            vesc_endpoint = argv[++i];
        } else if (arg == "--localization-endpoint" && i + 1 < argc) {
            localization_endpoint = argv[++i];
        }
    }

    std::cout << "[MPC " << robot_name << "] Launching with HandshakePort=" << handshake_port 
              << ", VESC=" << vesc_endpoint << ", Localization=" << localization_endpoint << std::endl;

    // ZeroMQ context
    zmq::context_t context(1);

    // 1. MARS Handshake Layer (REP socket)
    zmq::socket_t rep_socket(context, zmq::socket_type::rep);
    rep_socket.set(zmq::sockopt::linger, 0);
    std::string rep_addr = "tcp://*:" + std::to_string(handshake_port);
    try {
        rep_socket.bind(rep_addr);
    } catch (const std::exception& ex) {
        std::cerr << "[MPC " << robot_name << "] Failed to bind to REP port: " << ex.what() << std::endl;
        return 1;
    }

    // Phase 1 Handshake: Receive Trajectory
    zmq::message_t traj_msg;
    std::cout << "[MPC " << robot_name << "] Waiting for trajectory..." << std::endl;
    rep_socket.recv(traj_msg, zmq::recv_flags::none);
    std::string traj_encoded(static_cast<char*>(traj_msg.data()), traj_msg.size());

    // Decode and deserialize trajectory
    std::string traj_decoded = base64_decode(traj_encoded);
    ReloPush::trajectory ref_traj;
    ref_traj.deserialize(traj_decoded);
    std::cout << "[MPC " << robot_name << "] Received trajectory with " 
              << ref_traj.trajectory_points->size() << " waypoints. Sending ACK..." << std::endl;

    std::string ack_receive = "ACK_RECEIVE_" + robot_name;
    zmq::message_t ack_recv_msg(ack_receive.begin(), ack_receive.end());
    rep_socket.send(ack_recv_msg, zmq::send_flags::none);

    // Phase 2 Handshake: Receive START
    zmq::message_t start_msg;
    std::cout << "[MPC " << robot_name << "] Waiting for START signal..." << std::endl;
    rep_socket.recv(start_msg, zmq::recv_flags::none);
    std::string start_cmd(static_cast<char*>(start_msg.data()), start_msg.size());
    std::cout << "[MPC " << robot_name << "] Command received: " << start_cmd << ". Starting motion..." << std::endl;

    std::string ack_start = "ACK_START_" + robot_name;
    zmq::message_t ack_start_msg(ack_start.begin(), ack_start.end());
    rep_socket.send(ack_start_msg, zmq::send_flags::none);

    // 2. VESC Publisher (PUB socket)
    zmq::socket_t vesc_pub(context, zmq::socket_type::pub);
    vesc_pub.set(zmq::sockopt::linger, 0);
    try {
        // VESC controller is usually a SUB socket subscribing, so we connect or bind.
        // Let's connect by default, or support bind/connect if required. 
        // We will connect to the vesc_endpoint.
        vesc_pub.connect(vesc_endpoint);
        std::cout << "[MPC " << robot_name << "] VESC publisher connected to " << vesc_endpoint << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[MPC " << robot_name << "] Failed to connect VESC publisher: " << ex.what() << std::endl;
        return 1;
    }

    // 3. Localization Subscriber (SUB socket)
    zmq::socket_t loc_sub(context, zmq::socket_type::sub);
    loc_sub.set(zmq::sockopt::linger, 0);
    try {
        loc_sub.connect(localization_endpoint);
        // Subscribe to dummy localization topic
        std::string loc_topic = "/" + robot_name + "/localization";
        loc_sub.set(zmq::sockopt::subscribe, loc_topic);
        std::cout << "[MPC " << robot_name << "] Subscribed to localization topic: " << loc_topic << " at " << localization_endpoint << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[MPC " << robot_name << "] Failed to connect localization subscriber: " << ex.what() << std::endl;
        return 1;
    }

    // MPC parameters matching ReloPushMPC.py
    const double L = 0.29;          // Wheelbase
    const double dt = 0.05;         // Time step
    const int T = 8;                // Prediction horizon
    
    // Objective weights
    const double w_dist_default = 5.0;
    const double w_yaw = 1.0;
    const double w_vel = 0.2;
    const double w_lat = 40.0;
    const double w_control = 0.001;
    const double w_delta_rate = 50.0;

    // Velocity limits
    const double max_v_push = 0.295;
    const double max_v_nonpush = 0.38;
    const double max_steer = 0.33;
    const double max_accel = 0.73;

    // Scaling
    const double vel_scale = 1.01;
    const double vel_scale_back = 0.9;

    // State trackers
    PoseState state;
    PoseState current_pose;
    double last_cmd_v = 0.0;
    double last_cmd_delta = 0.0;
    double prev_steer = 0.0;

    auto last_loc_time = std::chrono::steady_clock::now();
    bool has_localization = false;

    double traj_start_time = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    std::cout << "[MPC " << robot_name << "] Entering control loop..." << std::endl;

    // Main Control Loop at 20Hz (dt = 0.05s)
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();
        double current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Check if trajectory is complete
        const auto& points = *(ref_traj.trajectory_points);
        double traj_end_time = traj_start_time + points.back().time + 0.3;
        double finish_waiting = 1.0;

        if (current_time > traj_end_time + finish_waiting) {
            // Command stop
            nlohmann::json payload;
            payload["speed"] = encodeAscii(0.0);
            payload["steering"] = encodeAscii(0.0);
            payload["accel"] = encodeAscii(0.0);
            std::string payload_str = payload.dump();

            std::string topic = "/" + robot_name + "/ackermann";
            zmq::message_t topic_msg(topic.begin(), topic.end());
            zmq::message_t payload_msg(payload_str.begin(), payload_str.end());

            vesc_pub.send(topic_msg, zmq::send_flags::sndmore);
            vesc_pub.send(payload_msg, zmq::send_flags::none);

            std::cout << "[MPC " << robot_name << "] Trajectory completed. Stopping vehicle and exiting." << std::endl;
            break;
        }

        // Receive localization updates (non-blocking)
        zmq::message_t msg;
        bool got_msg = false;
        std::string last_payload;

        while (loc_sub.recv(msg, zmq::recv_flags::dontwait)) {
            got_msg = true;
            std::string part(static_cast<char*>(msg.data()), msg.size());
            while (msg.more()) {
                loc_sub.recv(msg, zmq::recv_flags::none);
                part = std::string(static_cast<char*>(msg.data()), msg.size());
            }
            last_payload = part;
        }

        if (got_msg) {
            try {
                // If it is multi-part topic+payload, sometimes the last part is parsed.
                // Let's support both clean JSON and topic prefixed strings by checking format.
                if (last_payload.front() == '{') {
                    auto json_data = nlohmann::json::parse(last_payload);
                    current_pose.x = json_data.at("x").get<double>();
                    current_pose.y = json_data.at("y").get<double>();
                    current_pose.yaw = json_data.at("yaw").get<double>();
                    last_loc_time = std::chrono::steady_clock::now();
                    has_localization = true;
                }
            } catch (...) {
                // Ignore parsing errors for non-json or routing headers
            }
        }

        // Check if localization is available
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - last_loc_time).count();

        if (has_localization && elapsed_sec <= 0.1) {
            state = current_pose;
        } else {
            // Dead-reckoning fallback
            state.x += last_cmd_v * std::cos(state.yaw) * dt;
            state.y += last_cmd_v * std::sin(state.yaw) * dt;
            state.yaw += (last_cmd_v / L) * std::tan(last_cmd_delta) * dt;
        }

        // Delay compensation (predict 1 step ahead)
        PoseState pred_state = state;
        pred_state.x += last_cmd_v * std::cos(pred_state.yaw) * dt;
        pred_state.y += last_cmd_v * std::sin(pred_state.yaw) * dt;
        pred_state.yaw += (last_cmd_v / L) * std::tan(last_cmd_delta) * dt;

        // Determine weights and speeds
        double target_time = current_time + dt;
        auto [ref_x, ref_y, ref_yaw, ref_vel, is_pushing] = get_ref_state_at_time(target_time, traj_start_time, ref_traj);

        double w_dist = w_dist_default;
        bool change_dir = false;
        if ((last_cmd_v > 0 && ref_vel < 0) || (last_cmd_v < 0 && ref_vel > 0)) {
            w_dist = 50.0;
            if (last_cmd_v > 0 && ref_vel < 0) {
                change_dir = true;
            }
        }

        double max_v = is_pushing ? max_v_push : max_v_nonpush;

        // Ceres Optimization setup
        double u[2 * T];
        for (int t = 0; t < T; ++t) {
            u[2 * t] = last_cmd_v;
            u[2 * t + 1] = last_cmd_delta;
        }

        std::vector<MPCCostFunctor::RefState> horizon_refs;
        horizon_refs.reserve(T);
        for (int t = 0; t < T; ++t) {
            double h_time = current_time + (t + 1) * dt;
            auto [rx, ry, ryaw, rv, _] = get_ref_state_at_time(h_time, traj_start_time, ref_traj);
            horizon_refs.push_back({rx, ry, ryaw, rv});
        }

        ceres::Problem problem;
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<MPCCostFunctor, 63, 16>(
                new MPCCostFunctor(pred_state.x, pred_state.y, pred_state.yaw, L, dt, T,
                                   horizon_refs, w_dist, w_yaw, w_vel, w_lat, w_control, w_delta_rate));

        problem.AddResidualBlock(cost_function, nullptr, u);

        // Apply parameter bounds
        for (int t = 0; t < T; ++t) {
            problem.SetParameterLowerBound(u, 2 * t, -max_v);
            problem.SetParameterUpperBound(u, 2 * t, max_v);
            problem.SetParameterLowerBound(u, 2 * t + 1, -max_steer);
            problem.SetParameterUpperBound(u, 2 * t + 1, max_steer);
        }

        ceres::Solver::Options solver_options;
        solver_options.linear_solver_type = ceres::DENSE_QR;
        solver_options.max_num_iterations = 30;
        solver_options.logging_type = ceres::SILENT;
        solver_options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);

        double opt_v = u[0];
        double opt_delta = u[1];

        // Apply acceleration limit
        double max_delta_v = max_accel * dt;
        if (opt_v > last_cmd_v + max_delta_v) {
            opt_v = last_cmd_v + max_delta_v;
        } else if (opt_v < last_cmd_v - max_delta_v) {
            opt_v = last_cmd_v - max_delta_v;
        }

        // Apply scaling
        double scaled_speed = opt_v;
        if (opt_v < 0) {
            scaled_speed = vel_scale_back * opt_v;
        } else {
            scaled_speed = vel_scale * opt_v;
        }
        if (change_dir) {
            scaled_speed *= 0.77;
        }

        double accel = (opt_v - last_cmd_v) / dt;

        // Update tracking states
        last_cmd_v = opt_v;
        last_cmd_delta = opt_delta;
        prev_steer = opt_delta;

        // Package and send to VESC publisher
        nlohmann::json payload;
        payload["speed"] = encodeAscii(scaled_speed);
        payload["steering"] = encodeAscii(opt_delta);
        payload["accel"] = encodeAscii(accel);
        std::string payload_str = payload.dump();

        std::string topic = "/" + robot_name + "/ackermann";
        zmq::message_t topic_msg(topic.begin(), topic.end());
        zmq::message_t payload_msg(payload_str.begin(), payload_str.end());

        vesc_pub.send(topic_msg, zmq::send_flags::sndmore);
        vesc_pub.send(payload_msg, zmq::send_flags::none);

        // Synchronize loop speed to 20Hz (50ms interval)
        auto loop_end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        int sleep_ms = 50 - static_cast<int>(duration.count());
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    return 0;
}
