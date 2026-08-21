#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <cmath>

#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <ReloPush/base64.h>

// Base64 ASCII encoder matching main.cpp
std::string encodeAscii(double value) {
    std::ostringstream oss;
    oss << std::setprecision(16) << value;
    return base64_encode(oss.str());
}

int main(int argc, char** argv) {
    // Default testing configuration - placeholders for physical deployment
    std::string endpoint = "tcp://0.0.0.0:3160"; 
    std::string robot_name = "robot1";
    double target_speed = 0.2;      // m/s
    double target_distance = 1.0;   // meters
    double dt = 0.05;               // 20Hz control loop (50ms)
    bool use_bind = true;           // Default to bind to match vesc_zmq_sender

    // Command-line parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--endpoint" && i + 1 < argc) {
            endpoint = argv[++i];
        } else if (arg == "--robot" && i + 1 < argc) {
            robot_name = argv[++i];
        } else if (arg == "--speed" && i + 1 < argc) {
            target_speed = std::stod(argv[++i]);
        } else if (arg == "--distance" && i + 1 < argc) {
            target_distance = std::stod(argv[++i]);
        } else if (arg == "--bind") {
            use_bind = true;
        } else if (arg == "--connect") {
            use_bind = false;
        }
    }

    std::cout << "[Dummy MPC] Launching Straight Mover Test..." << std::endl;
    std::cout << "  VESC Endpoint:   " << endpoint << " (" << (use_bind ? "bind" : "connect") << ")" << std::endl;
    std::cout << "  Robot Name:      " << robot_name << std::endl;
    std::cout << "  Target Speed:    " << target_speed << " m/s" << std::endl;
    std::cout << "  Target Distance: " << target_distance << " meters" << std::endl;

    if (endpoint.find("<IP>") != std::string::npos || endpoint.find("<PORT>") != std::string::npos) {
        std::cerr << "[WARNING] Using placeholder VESC endpoint. Please specify a valid --endpoint <tcp://IP:PORT> to run." << std::endl;
    }

    // Initialize ZeroMQ context & socket
    zmq::context_t context(1);
    zmq::socket_t vesc_pub(context, zmq::socket_type::pub);
    vesc_pub.set(zmq::sockopt::linger, 0);

    try {
        if (use_bind) {
            vesc_pub.bind(endpoint);
            std::cout << "[Dummy MPC] Bound to VESC endpoint: " << endpoint << std::endl;
        } else {
            vesc_pub.connect(endpoint);
            std::cout << "[Dummy MPC] Connected to VESC endpoint: " << endpoint << std::endl;
        }
    } catch (const std::exception& ex) {
        std::cerr << "[Dummy MPC] Failed to bind/connect socket: " << ex.what() << std::endl;
        return 1;
    }


    // Short wait to ensure ZeroMQ socket is connected/registered before publishing
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string topic = "/" + robot_name + "/ackermann";
    std::cout << "[Dummy MPC] Publishing commands to topic: " << topic << " at 20Hz..." << std::endl;

    double elapsed_time = 0.0;
    double current_distance = 0.0;
    auto start_time = std::chrono::steady_clock::now();

    while (current_distance < target_distance) {
        auto loop_start = std::chrono::steady_clock::now();

        // Calculate distance traveled
        elapsed_time = std::chrono::duration<double>(loop_start - start_time).count();
        current_distance = std::abs(target_speed) * elapsed_time;

        std::cout << "\r[Dummy MPC] Progress: " << std::fixed << std::setprecision(2)
                  << current_distance << " / " << target_distance << " meters ("
                  << elapsed_time << "s elapsed)" << std::flush;

        // Command details
        double steer = 0.0;
        double accel = 0.0;

        // Package and serialize
        nlohmann::json payload;
        payload["speed"] = encodeAscii(target_speed);
        payload["steering"] = encodeAscii(steer);
        payload["accel"] = encodeAscii(accel);
        std::string payload_str = payload.dump();

        // Send ZeroMQ multipart message
        zmq::message_t topic_msg(topic.begin(), topic.end());
        zmq::message_t payload_msg(payload_str.begin(), payload_str.end());

        try {
            vesc_pub.send(topic_msg, zmq::send_flags::sndmore);
            vesc_pub.send(payload_msg, zmq::send_flags::none);
        } catch (const std::exception& ex) {
            std::cerr << "\n[Dummy MPC] ZeroMQ Send error: " << ex.what() << std::endl;
            break;
        }

        // Loop pacing: maintain 20Hz
        auto loop_end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        int sleep_ms = 50 - static_cast<int>(duration.count());
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    std::cout << "\n[Dummy MPC] Target distance reached. Initiating deceleration and stop sequence..." << std::endl;

    // Send Stop sequence repeatedly to ensure reliability
    for (int i = 0; i < 10; ++i) {
        nlohmann::json payload;
        payload["speed"] = encodeAscii(0.0);
        payload["steering"] = encodeAscii(0.0);
        payload["accel"] = encodeAscii(-1.5); // deceleration/brake
        std::string payload_str = payload.dump();

        zmq::message_t topic_msg(topic.begin(), topic.end());
        zmq::message_t payload_msg(payload_str.begin(), payload_str.end());

        try {
            vesc_pub.send(topic_msg, zmq::send_flags::sndmore);
            vesc_pub.send(payload_msg, zmq::send_flags::none);
        } catch (...) {}

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "[Dummy MPC] Straight move test complete. Exiting." << std::endl;
    return 0;
}
