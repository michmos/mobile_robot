#ifndef MOBILE_ROBOT_HARDWARE_TEST_FAKE_ESP32_HPP
#define MOBILE_ROBOT_HARDWARE_TEST_FAKE_ESP32_HPP

// plays the esp32 on the master side of a pty pair, see TESTING.md §4
//
// The plugin gets slavePath() as its serial_port parameter and talks to it
// with its unchanged production serial code. A reader thread collects the
// lines the plugin sends and, depending on the mode, answers config lines
// like the firmware would.

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/eventfd.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <filesystem>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace mobile_robot_hardware::test {

class FakeEsp32 {
public:
  // how the fake answers a "C,<id>,..." line
  enum class Mode {
    AUTO_ACK, // "S,ACK,<id>"
    NACK,     // "S,NACK,<id>,checksum", as for a corrupted line
    SILENT,   // no answer, e.g. a hung or unflashed esp32
  };

  explicit FakeEsp32(Mode mode = Mode::AUTO_ACK) : mode_(mode) {
    // raw from the start: in the default canonical mode the line discipline
    // would echo what the fake sends back to it before the plugin opened the
    // port (asio switches the port to raw on open)
    termios raw{};
    cfmakeraw(&raw);
    char name[128] = {};
    if (openpty(&master_, &slave_, name, &raw, nullptr) != 0) {
      throw std::runtime_error(std::string("openpty failed: ") +
                               std::strerror(errno));
    }
    slavePath_ = name;
    stopFd_ = eventfd(0, EFD_CLOEXEC);
    if (stopFd_ < 0) {
      closeFds_();
      throw std::runtime_error(std::string("eventfd failed: ") +
                               std::strerror(errno));
    }
    reader_ = std::thread([this] { readLoop_(); });
  }

  ~FakeEsp32() {
    // an eventfd write only fails on counter overflow, which one write can't
    // reach
    const uint64_t one = 1;
    [[maybe_unused]] const ssize_t ignored =
        ::write(stopFd_, &one, sizeof(one));
    reader_.join();
    closeFds_();
  }

  FakeEsp32(const FakeEsp32 &) = delete;
  FakeEsp32 &operator=(const FakeEsp32 &) = delete;

  // /dev/pts/N, hand it to the plugin as the serial_port parameter
  const std::string &slavePath() const { return slavePath_; }

  void setMode(Mode mode) { mode_ = mode; }

  // sends `line` + '\n' to the plugin, as the esp32 would
  void sendLine(const std::string &line) { writeAll_(line + "\n"); }

  // waits for the next received line that starts with `prefix`. Lines before
  // it are consumed as well, so consecutive calls walk forward through what
  // the plugin sent
  // @return: the line without '\n', nullopt if none arrived within `timeout`
  std::optional<std::string> waitForLine(const std::string &prefix,
                                         std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    std::optional<std::string> found;
    cv_.wait_for(lock, timeout, [&] {
      while (!lines_.empty()) {
        std::string line = std::move(lines_.front());
        lines_.pop_front();
        if (line.rfind(prefix, 0) == 0) {
          found = std::move(line);
          return true;
        }
      }
      return false;
    });
    return found;
  }

  // waits for the next received line, whatever it is
  std::optional<std::string> nextLine(std::chrono::milliseconds timeout) {
    return waitForLine("", timeout);
  }

  // forgets everything received so far
  void discardReceived() {
    std::lock_guard<std::mutex> lock(mutex_);
    lines_.clear();
  }

  // @return: true if anything other than the fake itself holds the slave side
  //          open, i.e. the plugin's serial port is open. The plugin runs in
  //          this process, so its file descriptors are listed in /proc/self/fd
  bool slaveOpenedByPlugin() const {
    namespace fs = std::filesystem;
    int count = 0;
    for (const auto &entry : fs::directory_iterator("/proc/self/fd")) {
      std::error_code ec;
      const fs::path target = fs::read_symlink(entry.path(), ec);
      if (!ec && target == slavePath_) {
        ++count;
      }
    }
    // the fake keeps its own descriptor open, see slave_
    return count > 1;
  }

  // the handshake id of a "C,<id>,..." line, 0 if it has none
  static uint32_t configId(const std::string &line) {
    if (line.rfind("C,", 0) != 0) {
      return 0;
    }
    return static_cast<uint32_t>(std::strtoul(line.c_str() + 2, nullptr, 10));
  }

private:
  void readLoop_() {
    std::string partial;
    char buf[256];
    while (true) {
      pollfd fds[2] = {{master_, POLLIN, 0}, {stopFd_, POLLIN, 0}};
      if (poll(fds, 2, -1) < 0) {
        if (errno == EINTR) {
          continue;
        }
        return;
      }
      if (fds[1].revents != 0) {
        return;
      }
      const ssize_t n = ::read(master_, buf, sizeof(buf));
      if (n <= 0) {
        // the fake holds the slave open, so this is not a hangup. Retry
        // unless the master side itself is broken
        if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
          continue;
        }
        return;
      }
      partial.append(buf, static_cast<size_t>(n));
      size_t nl;
      while ((nl = partial.find('\n')) != std::string::npos) {
        std::string line = partial.substr(0, nl);
        partial.erase(0, nl + 1);
        handleLine_(line);
      }
    }
  }

  void handleLine_(const std::string &line) {
    // recorded before answering: once the plugin got the ACK, e.g.
    // on_configure() returned, the test may discardReceived() and expects
    // the config line to be gone with it
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lines_.push_back(line);
    }
    cv_.notify_all();

    const uint32_t id = configId(line);
    if (id == 0) {
      return;
    }
    try {
      switch (mode_.load()) {
      case Mode::AUTO_ACK:
        sendLine("S,ACK," + std::to_string(id));
        break;
      case Mode::NACK:
        sendLine("S,NACK," + std::to_string(id) + ",checksum");
        break;
      case Mode::SILENT:
        break;
      }
    } catch (const std::exception &) {
      // an exception escaping the reader thread would abort the whole test
      // binary. The missing answer makes the waiting test fail instead
    }
  }

  void writeAll_(const std::string &data) {
    std::lock_guard<std::mutex> lock(writeMutex_);
    size_t done = 0;
    while (done < data.size()) {
      const ssize_t n =
          ::write(master_, data.data() + done, data.size() - done);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error(std::string("write to pty failed: ") +
                                 std::strerror(errno));
      }
      done += static_cast<size_t>(n);
    }
  }

  void closeFds_() {
    for (int fd : {stopFd_, slave_, master_}) {
      if (fd >= 0) {
        ::close(fd);
      }
    }
  }

  int master_ = -1;
  // kept open for the whole lifetime: otherwise the master side reads EIO
  // whenever the plugin has the port closed, and the raw settings could be
  // lost between two opens
  int slave_ = -1;
  int stopFd_ = -1;
  std::string slavePath_;
  std::atomic<Mode> mode_;
  std::thread reader_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::string> lines_;

  // sendLine() runs on the test thread and on the reader thread (auto answers)
  std::mutex writeMutex_;
};

} // namespace mobile_robot_hardware::test

#endif
