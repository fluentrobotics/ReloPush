#include "SerialPort.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace vesc {

namespace {

speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return B115200;
    }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(other.fd_), last_error_(std::move(other.last_error_)) {
    other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        close();
        fd_ = other.fd_;
        last_error_ = std::move(other.last_error_);
        other.fd_ = -1;
    }
    return *this;
}

bool SerialPort::open(const std::string& path, int baud) {
    close();

    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        last_error_ = "open('" + path + "'): " + std::strerror(errno);
        return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem control lines, enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;    // 8 data bits
    tty.c_cflag &= ~PARENB; // no parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
#endif
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // no software flow control

    // Purely non-blocking reads: return immediately with whatever is
    // available (possibly nothing). Combined with O_NONBLOCK on the fd
    // itself, this is what makes read_available() safe to poll from a
    // tight loop without ever stalling the caller.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
        ::close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);  // discard any stale bytes from before we opened

    // Best-effort: assert DTR/RTS. Some USB-CDC-ACM devices (including
    // ChibiOS-based VESC firmware) want DTR asserted before they consider
    // the port "open" from the host's perspective. GUARDED: TIOCMGET/
    // TIOCMSET fail with ENOTTY on a pty slave (ptys have no modem-control
    // lines at all), and this transport must behave identically on a real
    // tty and a pty slave -- so a failure here is silently ignored rather
    // than surfaced as an open() failure.
    int modem_bits = 0;
    if (ioctl(fd, TIOCMGET, &modem_bits) == 0) {
        modem_bits |= TIOCM_DTR | TIOCM_RTS;
        (void)ioctl(fd, TIOCMSET, &modem_bits);  // best-effort; ignore failure
    }

    fd_ = fd;
    last_error_.clear();
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::read_available(std::vector<uint8_t>* out) {
    if (fd_ < 0) {
        last_error_ = "read_available: not open";
        return -1;
    }
    uint8_t buf[512];
    int total = 0;
    for (;;) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            out->insert(out->end(), buf, buf + n);
            total += static_cast<int>(n);
            if (static_cast<size_t>(n) < sizeof(buf)) {
                break;  // short read -- almost certainly drained for now
            }
            continue;  // buffer was full; there may be more waiting
        }
        if (n == 0) {
            // With VMIN=0/VTIME=0 this means "nothing available right now",
            // NOT end-of-file (that distinction only applies to regular
            // files/pipes).
            break;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }
        if (errno == EINTR) {
            continue;
        }
        last_error_ = std::string("read: ") + std::strerror(errno);
        return (total > 0) ? total : -1;
    }
    return total;
}

bool SerialPort::write_all(const std::vector<uint8_t>& data, int timeout_ms) {
    return write_all(data.data(), data.size(), timeout_ms);
}

bool SerialPort::write_all(const uint8_t* data, size_t len, int timeout_ms) {
    if (fd_ < 0) {
        last_error_ = "write_all: not open";
        return false;
    }
    const bool has_deadline = timeout_ms > 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 0);

    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::write(fd_, data + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            int poll_ms = 50;
            if (has_deadline) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= deadline) {
                    last_error_ = "write_all: timed out waiting for writability";
                    return false;
                }
                const auto left_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
                poll_ms = static_cast<int>(std::min<long long>(50, std::max<long long>(1, left_ms)));
            }
            struct pollfd pfd;
            pfd.fd = fd_;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            poll(&pfd, 1, poll_ms);
            continue;
        }
        last_error_ = std::string("write: ") + std::strerror(errno);
        return false;
    }
    return true;
}

}  // namespace vesc
