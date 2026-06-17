#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include "base64.h"

namespace
{

    using json = nlohmann::json;

    enum class NumericEncoding
    {
        ASCII,
        Float32LE,
        Float32BE,
        Float64LE,
        Float64BE
    };

    struct Options
    {
        std::string endpoint = "tcp://0.0.0.0:3160";
        std::string robot_name = "racecar";
        std::string topic_override;
        NumericEncoding encoding = NumericEncoding::ASCII;
        bool bind = true;
    };

    void printUsage(const char *argv0)
    {
        std::cout << "Usage: " << argv0 << " [--endpoint <tcp://ip:port>] [--robot <name>]"
                  << " [--topic <full_topic>] [--encoding <ascii|float32_le|float32_be|float64_le|float64_be>]"
                  << " [--bind|--connect]" << "\n";
        std::cout << "Interactive input format:\n";
        std::cout << "  speed steering accel [current] [brake]\n";
        std::cout << "Examples:\n";
        std::cout << "  1.0 0.2 0.0\n";
        std::cout << "  0.0 0.0 -2.0\n";
        std::cout << "  0.0 0.0 0.0 4.0\n";
        std::cout << "Commands: help, quit\n";
    }

    NumericEncoding parseEncoding(const std::string &encoding)
    {
        if (encoding == "ascii")
        {
            return NumericEncoding::ASCII;
        }
        if (encoding == "float32_le")
        {
            return NumericEncoding::Float32LE;
        }
        if (encoding == "float32_be")
        {
            return NumericEncoding::Float32BE;
        }
        if (encoding == "float64_le")
        {
            return NumericEncoding::Float64LE;
        }
        if (encoding == "float64_be")
        {
            return NumericEncoding::Float64BE;
        }

        throw std::runtime_error("Unknown encoding: " + encoding);
    }

    std::string encodeAscii(double value)
    {
        std::ostringstream oss;
        oss << std::setprecision(16) << value;
        return base64_encode(oss.str());
    }

    std::string encodeFloat32(double value, bool little_endian)
    {
        float f = static_cast<float>(value);
        uint32_t raw = 0;
        std::memcpy(&raw, &f, sizeof(f));
        char bytes[4];

        if (little_endian)
        {
            bytes[0] = static_cast<char>(raw & 0xFF);
            bytes[1] = static_cast<char>((raw >> 8) & 0xFF);
            bytes[2] = static_cast<char>((raw >> 16) & 0xFF);
            bytes[3] = static_cast<char>((raw >> 24) & 0xFF);
        }
        else
        {
            bytes[3] = static_cast<char>(raw & 0xFF);
            bytes[2] = static_cast<char>((raw >> 8) & 0xFF);
            bytes[1] = static_cast<char>((raw >> 16) & 0xFF);
            bytes[0] = static_cast<char>((raw >> 24) & 0xFF);
        }

        return base64_encode(std::string(bytes, 4));
    }

    std::string encodeFloat64(double value, bool little_endian)
    {
        uint64_t raw = 0;
        std::memcpy(&raw, &value, sizeof(value));
        char bytes[8];

        if (little_endian)
        {
            bytes[0] = static_cast<char>(raw & 0xFF);
            bytes[1] = static_cast<char>((raw >> 8) & 0xFF);
            bytes[2] = static_cast<char>((raw >> 16) & 0xFF);
            bytes[3] = static_cast<char>((raw >> 24) & 0xFF);
            bytes[4] = static_cast<char>((raw >> 32) & 0xFF);
            bytes[5] = static_cast<char>((raw >> 40) & 0xFF);
            bytes[6] = static_cast<char>((raw >> 48) & 0xFF);
            bytes[7] = static_cast<char>((raw >> 56) & 0xFF);
        }
        else
        {
            bytes[7] = static_cast<char>(raw & 0xFF);
            bytes[6] = static_cast<char>((raw >> 8) & 0xFF);
            bytes[5] = static_cast<char>((raw >> 16) & 0xFF);
            bytes[4] = static_cast<char>((raw >> 24) & 0xFF);
            bytes[3] = static_cast<char>((raw >> 32) & 0xFF);
            bytes[2] = static_cast<char>((raw >> 40) & 0xFF);
            bytes[1] = static_cast<char>((raw >> 48) & 0xFF);
            bytes[0] = static_cast<char>((raw >> 56) & 0xFF);
        }

        return base64_encode(std::string(bytes, 8));
    }

    std::string encodeNumber(double value, NumericEncoding encoding)
    {
        switch (encoding)
        {
        case NumericEncoding::ASCII:
            return encodeAscii(value);
        case NumericEncoding::Float32LE:
            return encodeFloat32(value, true);
        case NumericEncoding::Float32BE:
            return encodeFloat32(value, false);
        case NumericEncoding::Float64LE:
            return encodeFloat64(value, true);
        case NumericEncoding::Float64BE:
            return encodeFloat64(value, false);
        default:
            throw std::runtime_error("Unsupported encoding");
        }
    }

    std::string buildTopic(const Options &options)
    {
        if (!options.topic_override.empty())
        {
            return options.topic_override;
        }
        return "/" + options.robot_name + "/ackermann";
    }

    bool parseLine(const std::string &line,
                   double *speed,
                   double *steering,
                   double *accel,
                   bool *has_current,
                   double *current,
                   bool *has_brake,
                   double *brake)
    {
        std::istringstream iss(line);
        if (!(iss >> *speed >> *steering >> *accel))
        {
            return false;
        }

        if (iss >> *current)
        {
            *has_current = true;
        }
        else
        {
            *has_current = false;
        }

        if (iss >> *brake)
        {
            *has_brake = true;
        }
        else
        {
            *has_brake = false;
        }

        return true;
    }

    Options parseArgs(int argc, char **argv)
    {
        Options options;
        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if (arg == "--endpoint" && i + 1 < argc)
            {
                options.endpoint = argv[++i];
            }
            else if (arg == "--robot" && i + 1 < argc)
            {
                options.robot_name = argv[++i];
            }
            else if (arg == "--topic" && i + 1 < argc)
            {
                options.topic_override = argv[++i];
            }
            else if (arg == "--encoding" && i + 1 < argc)
            {
                options.encoding = parseEncoding(argv[++i]);
            }
            else if (arg == "--connect")
            {
                options.bind = false;
            }
            else if (arg == "--bind")
            {
                options.bind = true;
            }
            else if (arg == "--help")
            {
                printUsage(argv[0]);
                std::exit(0);
            }
            else
            {
                throw std::runtime_error("Unknown argument: " + arg);
            }
        }
        return options;
    }

} // namespace

int main(int argc, char **argv)
{
    Options options;
    try
    {
        options = parseArgs(argc, argv);
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }

    const std::string topic = buildTopic(options);

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.set(zmq::sockopt::linger, 0);

    try
    {
        if (options.bind)
        {
            publisher.bind(options.endpoint);
            std::cout << "Publishing (bind) to " << options.endpoint << "\n";
        }
        else
        {
            publisher.connect(options.endpoint);
            std::cout << "Publishing (connect) to " << options.endpoint << "\n";
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Failed to setup publisher: " << ex.what() << "\n";
        return 1;
    }

    std::cout << "Topic: " << topic << "\n";
    std::cout << "Encoding: ";
    switch (options.encoding)
    {
    case NumericEncoding::ASCII:
        std::cout << "ascii";
        break;
    case NumericEncoding::Float32LE:
        std::cout << "float32_le";
        break;
    case NumericEncoding::Float32BE:
        std::cout << "float32_be";
        break;
    case NumericEncoding::Float64LE:
        std::cout << "float64_le";
        break;
    case NumericEncoding::Float64BE:
        std::cout << "float64_be";
        break;
    }
    std::cout << "\n";

    std::cout << "Enter commands (speed steering accel [current] [brake]) or 'help'/'quit'.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line.empty())
        {
            continue;
        }

        if (line == "quit" || line == "q")
        {
            break;
        }

        if (line == "help")
        {
            printUsage(argv[0]);
            continue;
        }

        double speed = 0.0;
        double steering = 0.0;
        double accel = 0.0;
        double current = 0.0;
        double brake = 0.0;
        bool has_current = false;
        bool has_brake = false;

        if (!parseLine(line, &speed, &steering, &accel, &has_current, &current, &has_brake, &brake))
        {
            std::cerr << "Invalid input. Format: speed steering accel [current] [brake]\n";
            continue;
        }

        json payload;
        payload["speed"] = encodeNumber(speed, options.encoding);
        payload["steering"] = encodeNumber(steering, options.encoding);
        payload["accel"] = encodeNumber(accel, options.encoding);
        if (has_current)
        {
            payload["current"] = encodeNumber(current, options.encoding);
        }
        if (has_brake)
        {
            payload["brake"] = encodeNumber(brake, options.encoding);
        }

        const std::string payload_str = payload.dump();

        zmq::message_t topic_msg(topic.begin(), topic.end());
        zmq::message_t payload_msg(payload_str.begin(), payload_str.end());
        publisher.send(topic_msg, zmq::send_flags::sndmore);
        publisher.send(payload_msg, zmq::send_flags::none);

        std::cout << "Sent topic=" << topic << " payload=" << payload_str << "\n";
    }

    return 0;
}
