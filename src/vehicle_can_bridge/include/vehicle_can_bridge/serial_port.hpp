// Minimal POSIX termios serial port wrapper for the vehicle_can_bridge ROS2 port.
//
// Design choices (Task 2 of the vehicle_can_bridge ROS1->ROS2 port plan):
//
//  * Namespace: `vehicle_can_bridge` (NOT `serial::Serial`). The class is
//    named `SerialPort` rather than reusing the original third-party
//    `serial::Serial` name, to avoid any ambiguity with the (removed)
//    external `serial` package this replaces. Nodes ported in later tasks
//    should write `vehicle_can_bridge::SerialPort serial_;`.
//
//  * Exception type: `vehicle_can_bridge::IOException`, derived from
//    `std::runtime_error`. It is thrown by `open()`, `write()`, `read()`
//    and `available()` on any underlying POSIX failure (open/tcgetattr/
//    tcsetattr/write/read/poll/ioctl errors), mirroring the original
//    `serial::IOException` call sites. Ported node code should change
//    `catch (serial::IOException& e)` to
//    `catch (vehicle_can_bridge::IOException& e)` — `e.what()` behaves the
//    same way (returns a descriptive message).
//
//  * `setTimeout()` takes a single millisecond value (no `serial::Timeout`
//    object) since callers only ever used
//    `serial::Timeout::simpleTimeout(1000)`. The timeout bounds how long
//    `read()`/`write()` will wait for I/O readiness via `poll()`; it does
//    not use termios VMIN/VTIME (the fd is opened O_NONBLOCK and blocking
//    behavior is implemented in userspace with poll()), which keeps
//    behavior identical regardless of how many bytes are requested.
//
//  * Baud rate: only the standard POSIX `B*` constants are supported
//    (including `B2000000`, confirmed available on the target toolchain/
//    kernel via <termios.h> — no termios2/BOTHER fallback needed). This
//    covers the package's real usage (default 2000000 via ROS param,
//    overridable to any other standard rate).
//
// All methods are defined inline in the class body; this header is safe to
// include from multiple translation units.

#ifndef VEHICLE_CAN_BRIDGE_SERIAL_PORT_HPP_
#define VEHICLE_CAN_BRIDGE_SERIAL_PORT_HPP_

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace vehicle_can_bridge
{

// Thrown for any serial I/O failure (open/configure/write/read/ioctl).
// Mirrors the original serial::IOException catch sites.
class IOException : public std::runtime_error
{
public:
  explicit IOException(const std::string & msg)
  : std::runtime_error(msg)
  {}
};

// Minimal POSIX termios-based stand-in for the third-party serial::Serial
// class, covering only the methods actually used by the vehicle_can_bridge
// nodes: setPort, setBaudrate, setTimeout, open, close, isOpen, write,
// read, available, flushInput, flushOutput.
class SerialPort
{
public:
  SerialPort() = default;

  ~SerialPort()
  {
    if (isOpen()) {
      close();
    }
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void setPort(const std::string & port)
  {
    port_ = port;
  }

  void setBaudrate(int baudrate)
  {
    baudrate_ = baudrate;
  }

  // Original call sites only ever used serial::Timeout::simpleTimeout(ms);
  // this collapses that to a single millisecond value used to bound
  // poll()-based waits in read()/write().
  void setTimeout(int timeout_ms)
  {
    timeout_ms_ = timeout_ms;
  }

  void open()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw IOException(
        "Failed to open serial port " + port_ + ": " + std::strerror(errno));
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      const int err = errno;
      closeFd();
      throw IOException(
        "tcgetattr failed on " + port_ + ": " + std::strerror(err));
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;   // no parity
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no hw flow control
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 data bits
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // no sw flow control

    // Reads are driven by poll() with our own timeout, not VMIN/VTIME.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const speed_t speed = baudToSpeed(baudrate_);
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
      const int err = errno;
      closeFd();
      throw IOException(
        "Failed to set baudrate " + std::to_string(baudrate_) +
        " on " + port_ + ": " + std::strerror(err));
    }

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const int err = errno;
      closeFd();
      throw IOException(
        "tcsetattr failed on " + port_ + ": " + std::strerror(err));
    }

    tcflush(fd_, TCIOFLUSH);
  }

  void close()
  {
    closeFd();
  }

  bool isOpen() const
  {
    return fd_ >= 0;
  }

  size_t write(const std::vector<uint8_t> & data)
  {
    requireOpen("write");

    size_t total_written = 0;
    while (total_written < data.size()) {
      const ssize_t n = ::write(
        fd_, data.data() + total_written, data.size() - total_written);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          struct pollfd pfd;
          pfd.fd = fd_;
          pfd.events = POLLOUT;
          pfd.revents = 0;
          const int pr = ::poll(&pfd, 1, timeout_ms_);
          if (pr <= 0) {
            throw IOException("write() timed out on " + port_);
          }
          continue;
        }
        throw IOException(
          std::string("write() failed on ") + port_ + ": " + std::strerror(errno));
      }
      total_written += static_cast<size_t>(n);
    }
    return total_written;
  }

  // Reads up to max_len bytes, waiting (via poll) up to the configured
  // timeout for data to arrive. Returns whatever was accumulated before
  // max_len bytes were read or the timeout elapsed (possibly empty),
  // mirroring the original serial_.read(available)/serial_.read(100)
  // call sites, which do not require the exact byte count requested.
  std::string read(size_t max_len)
  {
    requireOpen("read");

    std::string result;
    result.reserve(max_len);

    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);

    while (result.size() < max_len) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      const int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
      if (remaining_ms <= 0) {
        break;
      }

      struct pollfd pfd;
      pfd.fd = fd_;
      pfd.events = POLLIN;
      pfd.revents = 0;
      const int pr = ::poll(&pfd, 1, remaining_ms);
      if (pr < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw IOException(
          std::string("read() poll failed on ") + port_ + ": " + std::strerror(errno));
      }
      if (pr == 0) {
        break;  // timed out; return whatever we have (possibly empty)
      }

      char buf[256];
      const size_t chunk = std::min(sizeof(buf), max_len - result.size());
      const ssize_t n = ::read(fd_, buf, chunk);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        throw IOException(
          std::string("read() failed on ") + port_ + ": " + std::strerror(errno));
      }
      if (n == 0) {
        break;  // EOF / device disconnected
      }
      result.append(buf, static_cast<size_t>(n));
    }
    return result;
  }

  size_t available()
  {
    if (!isOpen()) {
      return 0;
    }
    int n = 0;
    if (::ioctl(fd_, FIONREAD, &n) != 0) {
      throw IOException(
        std::string("ioctl(FIONREAD) failed on ") + port_ + ": " + std::strerror(errno));
    }
    return n > 0 ? static_cast<size_t>(n) : 0;
  }

  void flushInput()
  {
    if (isOpen()) {
      tcflush(fd_, TCIFLUSH);
    }
  }

  void flushOutput()
  {
    if (isOpen()) {
      tcflush(fd_, TCOFLUSH);
    }
  }

private:
  void closeFd()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void requireOpen(const char * op) const
  {
    if (!isOpen()) {
      throw IOException(
        std::string(op) + "() called on closed serial port " + port_);
    }
  }

  static speed_t baudToSpeed(int baudrate)
  {
    switch (baudrate) {
      case 1200: return B1200;
      case 1800: return B1800;
      case 2400: return B2400;
      case 4800: return B4800;
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 500000: return B500000;
      case 576000: return B576000;
      case 921600: return B921600;
      case 1000000: return B1000000;
      case 1152000: return B1152000;
      case 1500000: return B1500000;
      case 2000000: return B2000000;
      case 2500000: return B2500000;
      case 3000000: return B3000000;
      case 3500000: return B3500000;
      case 4000000: return B4000000;
      default:
        throw IOException("Unsupported baudrate: " + std::to_string(baudrate));
    }
  }

  int fd_ = -1;
  std::string port_;
  int baudrate_ = 0;
  int timeout_ms_ = 1000;
};

}  // namespace vehicle_can_bridge

#endif  // VEHICLE_CAN_BRIDGE_SERIAL_PORT_HPP_
