// SerialPort.h
//
// Minimal POSIX termios wrapper around a serial device. This is a raw
// bytes-in/bytes-out transport -- framing/CRC/command semantics live in
// VescProtocol.h, not here. Deliberately built to work IDENTICALLY on a
// real tty (e.g. /dev/ttyACM0, a USB-CDC-ACM VESC) and on a pty slave (as
// created by tests/fake_vesc.cpp's posix_openpt()): no ioctls that fail on
// ptys are required for correct operation, and the one convenience ioctl
// this file does attempt (asserting DTR/RTS -- see .cpp) is guarded so its
// failure on a pty (ENOTTY there, since ptys have no modem control lines)
// is silently ignored rather than treated as an open() failure.
//
// Portability: C++14 only, POSIX termios/fcntl/poll only -- see
// VescDriver/CMakeLists.txt's HARD PORTABILITY RULES.

#ifndef VESC_DRIVER_SERIAL_PORT_H_
#define VESC_DRIVER_SERIAL_PORT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vesc {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    // Opens `path` with O_RDWR|O_NOCTTY|O_NONBLOCK, puts it into raw 8N1
    // mode (cfmakeraw) at `baud` with no hardware/software flow control and
    // VMIN=0/VTIME=0 (purely non-blocking reads -- read() returns
    // immediately with however many bytes are currently available, 0
    // meaning "none right now", not EOF). Closes any previously-open fd
    // first. Returns true on success; on failure returns false and
    // last_error() describes why.
    bool open(const std::string& path, int baud = 115200);

    void close();
    bool is_open() const { return fd_ >= 0; }

    // Non-blocking read: appends whatever is currently available (0 or
    // more bytes) to `*out` and returns the number of bytes appended.
    // Returns -1 on a hard read error (not "nothing available right now").
    int read_available(std::vector<uint8_t>* out);

    // Writes all of `data`, retrying on EINTR/EAGAIN with a short poll()
    // wait for writability in between. `timeout_ms` <= 0 means wait
    // forever for space to become available; otherwise gives up (and
    // returns false) once that many milliseconds have elapsed without
    // finishing the write. Returns true iff every byte was written.
    bool write_all(const std::vector<uint8_t>& data, int timeout_ms = 2000);
    bool write_all(const uint8_t* data, size_t len, int timeout_ms = 2000);

    const std::string& last_error() const { return last_error_; }

private:
    int fd_ = -1;
    std::string last_error_;
};

}  // namespace vesc

#endif  // VESC_DRIVER_SERIAL_PORT_H_
